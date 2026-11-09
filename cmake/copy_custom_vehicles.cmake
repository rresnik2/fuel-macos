# CUSTOM VEHICLES (port deviation): copies the custom vehicle packs (built outside the repo, see
# src/core/custom_vehicles.hpp) into FUEL.app/Contents/Resources/vehicles. Only complete packs (vehicle.tsc + the
# archive it names) and only their game files (vehicle.tsc, *.DPC, CREDITS.txt); source models and work files stay out.
# Run by the FUEL_app target: cmake -DSRC=.. -DDST=.. -P
file(REMOVE_RECURSE "${DST}")
set(_n 0)
if(IS_DIRECTORY "${SRC}")
  file(GLOB _packs LIST_DIRECTORIES true "${SRC}/*")
  foreach(_p ${_packs})
    if(IS_DIRECTORY "${_p}" AND EXISTS "${_p}/vehicle.tsc")
      file(GLOB _dpc "${_p}/*.DPC")
      if(_dpc)
        get_filename_component(_name "${_p}" NAME)
        file(GLOB _files "${_p}/vehicle.tsc" "${_p}/*.DPC" "${_p}/CREDITS.txt")
        file(MAKE_DIRECTORY "${DST}/${_name}")
        file(COPY ${_files} DESTINATION "${DST}/${_name}")
        math(EXPR _n "${_n} + 1")
      endif()
    endif()
  endforeach()
endif()
message(STATUS "custom vehicles: ${_n} pack(s) from ${SRC}")
