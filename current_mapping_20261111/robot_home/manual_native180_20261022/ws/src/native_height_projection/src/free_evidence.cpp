// Free observations only. Occupied cells always come from the native ikd-tree.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {
struct Key {
  int32_t x, y;
  bool operator==(const Key &b) const { return x == b.x && y == b.y; }
};
struct Hash {
  size_t operator()(const Key &p) const {
    return std::hash<uint64_t>()((uint64_t(uint32_t(p.x)) << 32) | uint32_t(p.y));
  }
};
struct Evidence {
  double resolution, low, high, dz, blind, guard;
  int layers;
  uint32_t complete;
  int32_t x0=0, y0=0, width=0, height=0;
  uint64_t rays=0;
  std::unordered_map<Key, uint32_t, Hash> masks;

  Evidence(double r, double l, double h, double z, double b, double g)
    : resolution(r), low(l), high(h), dz(z), blind(b), guard(g) {
    layers = int(std::ceil((high-low)/dz - 1e-9));
    complete = (uint32_t(1) << layers) - 1;
  }
  bool inside(Key k) const {
    return k.x >= x0 && k.y >= y0 && int64_t(k.x) < int64_t(x0)+width &&
           int64_t(k.y) < int64_t(y0)+height;
  }
  void bounds(int32_t x, int32_t y, int32_t w, int32_t h) {
    x0=x; y0=y; width=w; height=h;
    for (auto it=masks.begin(); it!=masks.end();) {
      if (!inside(it->first)) it=masks.erase(it); else ++it;
    }
  }
  void ray(const double *origin, const float *endpoint) {
    double d[3], length=0;
    for (int a=0; a<3; ++a) {
      if (!std::isfinite(origin[a]) || !std::isfinite(endpoint[a])) return;
      d[a]=double(endpoint[a])-origin[a]; length+=d[a]*d[a];
    }
    length=std::sqrt(length);
    if (length <= blind+guard || width<=0 || height<=0) return;
    double t0=blind/length, t1=1.0-guard/length;
    const double lo[3]={x0*resolution,y0*resolution,low};
    const double hi[3]={(int64_t(x0)+width)*resolution,
                        (int64_t(y0)+height)*resolution,high};
    for (int a=0; a<3; ++a) {
      if (std::abs(d[a])<1e-14) {
        if (origin[a]<lo[a] || origin[a]>=hi[a]) return;
      } else {
        double near=(lo[a]-origin[a])/d[a], far=(hi[a]-origin[a])/d[a];
        if (near>far) std::swap(near,far);
        t0=std::max(t0,near); t1=std::min(t1,far);
      }
    }
    if (!(t1-t0>1e-10)) return;
    ++rays;
    const double spacing[3]={resolution,resolution,dz};
    const double offset[3]={0,0,low};
    int32_t cell[3]; int step[3]; double next[3], delta[3];
    // Move just inside the clipped segment, including for negative directions.
    const double start=t0+std::min(1e-9,(t1-t0)*0.25);
    for (int a=0; a<3; ++a) {
      cell[a]=int32_t(std::floor((origin[a]+d[a]*start-offset[a])/spacing[a]));
      step[a]=(d[a]>0)-(d[a]<0);
      if (step[a]==0) next[a]=delta[a]=std::numeric_limits<double>::infinity();
      else {
        const double edge=offset[a]+(cell[a]+(step[a]>0 ? 1 : 0))*spacing[a];
        next[a]=(edge-origin[a])/d[a]; delta[a]=spacing[a]/std::abs(d[a]);
      }
    }
    for (;;) {
      Key k{cell[0],cell[1]};
      if (inside(k) && cell[2]>=0 && cell[2]<layers)
        masks[k] |= uint32_t(1) << cell[2];
      const double t=std::min({next[0],next[1],next[2]});
      if (!(t<t1-1e-12)) break;
      // A corner/edge touch is not evidence for the adjacent zero-length cells.
      for (int a=0; a<3; ++a) if (next[a]<=t+1e-12) {
        cell[a]+=step[a]; next[a]+=delta[a];
      }
    }
  }
};
}

extern "C" {
void *fe_create(double r,double low,double high,double dz,double blind,double guard) {
  const double n=(high-low)/dz;
  if (!std::isfinite(r+low+high+dz+blind+guard+n) || r<=0 || dz<=0 ||
      high<=low || n>31 || blind<0 || guard<0) return nullptr;
  try { return new Evidence(r,low,high,dz,blind,guard); } catch (...) { return nullptr; }
}
void fe_destroy(void *p) { delete static_cast<Evidence *>(p); }
void fe_clear(void *p) { if (p) static_cast<Evidence *>(p)->masks.clear(); }
int fe_bounds(void *p,int32_t x,int32_t y,int32_t w,int32_t h) {
  if (!p || w<=0 || h<=0) return -1;
  static_cast<Evidence *>(p)->bounds(x,y,w,h); return 0;
}
int fe_integrate(void *p,const float *xyz,int64_t count,const double *origin) {
  if (!p || !xyz || !origin || count<0) return -1;
  auto &e=*static_cast<Evidence *>(p);
  try {
    for (int64_t i=0; i<count; ++i) e.ray(origin,xyz+3*i);
    // Raw endpoints in the height band veto old free evidence for their column,
    // even when native downsampling has not kept that particular endpoint.
    for (int64_t i=0; i<count; ++i) {
      const float *q=xyz+3*i;
      if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2])) continue;
      if (q[2]>=e.low-1e-6 && q[2]<=e.high+1e-6) {
        const double x=std::floor(q[0]/e.resolution),y=std::floor(q[1]/e.resolution);
        if (x>=e.x0 && x<double(e.x0)+e.width && y>=e.y0 && y<double(e.y0)+e.height)
          e.masks.erase(Key{int32_t(x),int32_t(y)});
      }
    }
    return 0;
  } catch (...) { e.masks.clear(); return -2; } // fail unknown, never fabricate free
}
int64_t fe_apply(void *p,int8_t *grid) {
  if (!p || !grid) return -1;
  auto &e=*static_cast<Evidence *>(p); int64_t free=0;
  for (auto it=e.masks.begin(); it!=e.masks.end();) {
    const Key k=it->first;
    const int64_t index=int64_t(k.y-e.y0)*e.width+(k.x-e.x0);
    if (grid[index]==100) it=e.masks.erase(it);
    else { if (it->second==e.complete) { grid[index]=0; ++free; } ++it; }
  }
  return free;
}
int64_t fe_size(void *p) { return p ? static_cast<Evidence *>(p)->masks.size() : 0; }
uint64_t fe_rays(void *p) { return p ? static_cast<Evidence *>(p)->rays : 0; }
int64_t fe_export(void *p,int32_t *xy,uint32_t *masks,int64_t capacity) {
  if (!p || capacity<fe_size(p)) return -1;
  int64_t i=0;
  for (const auto &v:static_cast<Evidence *>(p)->masks) {
    xy[2*i]=v.first.x; xy[2*i+1]=v.first.y; masks[i]=v.second; ++i;
  }
  return i;
}
}
