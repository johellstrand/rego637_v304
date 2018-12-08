#!/bin/bash

gcc -Wall -std=c99  -Werror-implicit-function-declaration  rego.c -o bin/rego
if [ $? -eq 0 ]; then
	echo "Success!, bin/rego executable updated";
else 
	echo "Failure!!!!"; 
fi

gcc -Wall -std=c99  -Werror-implicit-function-declaration rego_modify_heatcurve.c -o bin/rego_modify_heatcurve
if [ $? -eq 0 ]; then
        echo "Success!, bin/rego_modify_heatcurve executable updated";
else
        echo "Failure!!!!";
fi
