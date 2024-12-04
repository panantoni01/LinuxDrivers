#!/bin/bash

TOOL=clang-format-14

FORMAT_DIRS=$(echo -n driver_{calc,litex_gpio,si7021})

MODIFY=-n
if [ "$1" == "--fix" ]; then
  MODIFY=""
fi

$TOOL $MODIFY -style=file --Werror -i $(find $FORMAT_DIRS -maxdepth 1 -name "*.[ch]" )

if [ $? -ne 0 ]; then
  echo "Incorrect formatting!"
  exit 1
fi

echo "Format check OK"

