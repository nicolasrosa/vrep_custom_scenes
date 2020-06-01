if(CMAKE_HAVE_SPROC_H AND NOT CMAKE_THREAD_PREFER_PTHREAD)
  # We have sproc
  set(CMAKE_USE_SPROC_INIT 1)
else()
  # Do we have pthreads?
  CHECK_INCLUDE_FILES("pthread.h" CMAKE_HAVE_PTHREAD_H)
  if(CMAKE_HAVE_PTHREAD_H)

    #
    # We have pthread.h
    # Let's check for the library now.
    #
    set(CMAKE_HAVE_THREADS_LIBRARY)
    if(NOT THREADS_HAVE_PTHREAD_ARG)
      # Check if pthread functions are in normal C library
      CHECK_SYMBOL_EXISTS(pthread_create pthread.h CMAKE_HAVE_LIBC_CREATE)
      if(CMAKE_HAVE_LIBC_CREATE)
        set(CMAKE_THREAD_LIBS_INIT "")
        set(CMAKE_HAVE_THREADS_LIBRARY 1)
        set(Threads_FOUND TRUE)
      endif()

      if(NOT CMAKE_HAVE_THREADS_LIBRARY)
        # Do we have -lpthreads
        CHECK_LIBRARY_EXISTS(pthreads pthread_create "" CMAKE_HAVE_PTHREADS_CREATE)
        if(CMAKE_HAVE_PTHREADS_CREATE)
          set(CMAKE_THREAD_LIBS_INIT "-lpthreads")
          set(CMAKE_HAVE_THREADS_LIBRARY 1)
          set(Threads_FOUND TRUE)
        endif()

        # Ok, how about -lpthread
        CHECK_LIBRARY_EXISTS(pthread pthread_create "" CMAKE_HAVE_PTHREAD_CREATE)
        if(CMAKE_HAVE_PTHREAD_CREATE)
          set(CMAKE_THREAD_LIBS_INIT "-lpthread")
          set(CMAKE_HAVE_THREADS_LIBRARY 1)
          set(Threads_FOUND TRUE)
        endif()

        if(CMAKE_SYSTEM MATCHES "SunOS.*")
          # On sun also check for -lthread
          CHECK_LIBRARY_EXISTS(thread thr_create "" CMAKE_HAVE_THR_CREATE)
          if(CMAKE_HAVE_THR_CREATE)
            set(CMAKE_THREAD_LIBS_INIT "-lthread")
            set(CMAKE_HAVE_THREADS_LIBRARY 1)
            set(Threads_FOUND TRUE)
          endif()
        endif()
      endif()
    endif()

    if(NOT CMAKE_HAVE_THREADS_LIBRARY)
      # If we did not found -lpthread, -lpthread, or -lthread, look for -pthread
      if("THREADS_HAVE_PTHREAD_ARG" MATCHES "^THREADS_HAVE_PTHREAD_ARG")
        message(STATUS "Check if compiler accepts -pthread")
        try_run(THREADS_PTHREAD_ARG THREADS_HAVE_PTHREAD_ARG
          ${CMAKE_BINARY_DIR}
          ${CMAKE_ROOT}/Modules/CheckForPthreads.c
          CMAKE_FLAGS -DLINK_LIBRARIES:STRING=-pthread
          COMPILE_OUTPUT_VARIABLE OUTPUT)

        if(THREADS_HAVE_PTHREAD_ARG)
          if(THREADS_PTHREAD_ARG STREQUAL "2")
            set(Threads_FOUND TRUE)
            message(STATUS "Check if compiler accepts -pthread - yes")
          else()
            message(STATUS "Check if compiler accepts -pthread - no")
            file(APPEND
              ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
              "Determining if compiler accepts -pthread returned ${THREADS_PTHREAD_ARG} instead of 2. The compiler had the following output:\n${OUTPUT}\n\n")
          endif()
        else()
          message(STATUS "Check if compiler accepts -pthread - no")
          file(APPEND
            ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Determining if compiler accepts -pthread failed with the following output:\n${OUTPUT}\n\n")
        endif()

      endif()

      if(THREADS_HAVE_PTHREAD_ARG)
        set(Threads_FOUND TRUE)
        set(CMAKE_THREAD_LIBS_INIT "-pthread")
      endif()

    endif()
  endif()
endif()