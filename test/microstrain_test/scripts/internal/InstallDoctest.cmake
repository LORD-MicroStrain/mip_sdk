include(FetchContent)

set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)

FetchContent_Declare(doctest
    GIT_REPOSITORY "https://github.com/doctest/doctest.git"
    GIT_TAG "v2.4.12"
)

FetchContent_MakeAvailable(doctest)
