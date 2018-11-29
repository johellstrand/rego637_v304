#!/bin/bash

gcc -Wall -std=c99  -Werror-implicit-function-declaration  rego.c -o bin/rego
if [ $? -eq 0 ]; then
	echo "Success!, bin/rego executable updated";
else 
	echo "Failure!!!!"; 
fi
