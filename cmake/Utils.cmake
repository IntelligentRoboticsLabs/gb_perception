# Function to download models
function(download_model MODEL_NAME MODEL_DOWNLOAD_FLAG MODEL_RELATIVE_PATH CHECKSUM)
  if(MODEL_DOWNLOAD_FLAG)
    message(STATUS "Downloading ${MODEL_NAME} model...")
    set(MODEL_FILENAME share/${PROJECT_NAME}/models/${MODEL_RELATIVE_PATH})
    if(NOT EXISTS ${MODEL_FILENAME})
      message(STATUS "NOTE: This process might take several minutes depending on your internet connection.")
      file(DOWNLOAD ${OPENPOSE_URL}${MODEL_RELATIVE_PATH} ${MODEL_FILENAME}
          EXPECTED_MD5 ${CHECKSUM}) # SHOW_PROGRESS)
    else(NOT EXISTS ${MODEL_FILENAME})
      message(STATUS "Model already exists.")
    endif(NOT EXISTS ${MODEL_FILENAME})
  else(MODEL_DOWNLOAD_FLAG)
    message(STATUS "Not downloading ${MODEL_NAME} model")
  endif(MODEL_DOWNLOAD_FLAG)
endfunction(download_model)

# Function to download zip files, then extracting them and then deleting them
function(download_zip FILE_NAME URL DOWNLOAD_PATH CHECKSUM)
  set(FULL_FILE_PATH "${DOWNLOAD_PATH}/${FILE_NAME}")
  if(NOT EXISTS ${FULL_FILE_PATH})
    message(STATUS "Downloading ${URL}/${FILE_NAME}...")
    file(DOWNLOAD "${URL}/${FILE_NAME}" "${DOWNLOAD_PATH}/${FILE_NAME}"
        EXPECTED_MD5 ${CHECKSUM})
    message(STATUS "Extracting ${FULL_FILE_PATH}...")
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xf ${FILE_NAME} WORKING_DIRECTORY ${DOWNLOAD_PATH})
    else(NOT EXISTS ${FULL_FILE_PATH})
      message(STATUS "${FILE_NAME} already exists.")
  endif(NOT EXISTS ${FULL_FILE_PATH})
endfunction(download_zip)

# Function to prepend filenames with common path
function(prepend var prefix)
  set(listVar "")
  foreach(f ${ARGN})
    list(APPEND listVar "${prefix}/${f}")
  endforeach(f)
  set(${var} "${listVar}" PARENT_SCOPE)
endfunction(prepend)

# Get names of subdirectories in directory
macro(subdirlist result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child} AND NOT ${child} STREQUAL "CMakeFiles")
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()