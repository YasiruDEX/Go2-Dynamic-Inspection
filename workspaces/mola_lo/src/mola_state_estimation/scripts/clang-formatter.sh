#!/bin/bash

find mola*/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | xargs -I FIL bash -c "echo FIL && clang-format-14 -i FIL"
