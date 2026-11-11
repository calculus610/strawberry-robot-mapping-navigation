# This hook is loaded by zzj_ws/install/setup.bash.
#
# The shared wheeltec account sources wheeltec_ros2 and Autoware for other
# teams. wheeltec_ros2 contains a second, independently built Nav2 stack.
# Remove only those duplicate Nav2-related path entries when a terminal
# explicitly selects zzj_ws; keep every non-Nav2 package from the shared
# overlays available.

_zzj_nav2_is_conflicting_path()
{
  case "$1" in
    */install/nav2_* | \
    */install/dwb_* | \
    */install/nav_2d_* | \
    */install/costmap_queue | \
    */install/costmap_queue/* | \
    */install/navigation2 | \
    */install/navigation2/*)
      return 0
      ;;
  esac
  return 1
}

_zzj_nav2_filter_colon_variable()
{
  local variable_name="$1"
  local original_value="${!variable_name-}"
  local filtered_value=""
  local entry
  local -a entries

  IFS=':' read -r -a entries <<< "${original_value}"
  for entry in "${entries[@]}"; do
    if _zzj_nav2_is_conflicting_path "${entry}"; then
      continue
    fi
    if [ -n "${filtered_value}" ]; then
      filtered_value="${filtered_value}:${entry}"
    else
      filtered_value="${entry}"
    fi
  done

  printf -v "${variable_name}" '%s' "${filtered_value}"
  export "${variable_name}"
}

for _zzj_nav2_variable in \
  AMENT_PREFIX_PATH \
  CMAKE_PREFIX_PATH \
  LD_LIBRARY_PATH \
  PYTHONPATH \
  PATH \
  PKG_CONFIG_PATH \
  CPATH \
  LIBRARY_PATH
do
  _zzj_nav2_filter_colon_variable "${_zzj_nav2_variable}"
done

export ZZJ_NAV2_RUNTIME_PREFIX="/opt/ros/humble"

unset _zzj_nav2_variable
unset -f _zzj_nav2_filter_colon_variable
unset -f _zzj_nav2_is_conflicting_path
