#!/bin/bash

find ./include -regex '.*\.\(h\|hpp\)' -exec clang-format -style=file -i {} \;

find ./src -regex '.*\.\(cpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;