#!/bin/bash

gcc -Wall -std=c99  -Werror-implicit-function-declaration  rego.c -o bin/rego
if [ $? -eq 0 ]; then
	echo "Success!, bin/rego executable updated";
else 
	echo "Failure!!!!"; 
fi

rm *.o
gcc -Wall -std=c99  -Werror-implicit-function-declaration -c rego_funcs.c
gcc -Wall -std=c99  -Werror-implicit-function-declaration -c rego_modify_heatcurve.c
gcc -Wall -std=c99  -Werror-implicit-function-declaration -o bin/rego_modify_heatcurve rego_funcs.o rego_modify_heatcurve.o
if [ $? -eq 0 ]; then
        echo "Success!, bin/rego_modify_heatcurve executable updated";
else
        echo "Failure!!!!";
fi

gcc -Wall -std=c99  -Werror-implicit-function-declaration -c fake_osx_rego_server.c
gcc -Wall -std=c99  -Werror-implicit-function-declaration -o bin/fake_osx_rego_server rego_funcs.o fake_osx_rego_server.o
if [ $? -eq 0 ]; then
        echo "Success!, bin/fake_osx_rego_server executable updated";
else
        echo "Failure!!!!";
fi
