#!/bin/bash

# Change the current working directory to the location of the present file
cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cmake -DINSTALL_GTEST=0 -Dgtest_disable_pthreads=1 ../googletest
make

