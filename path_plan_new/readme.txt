./exec_getbuildings.py -i 'Take 2022-11-08 08.26.58 AM.csv' -o outputfromtake.csv
or
./exec_getbuildings.py -o outputfromnatnet.csv


./exec_genmatrix.py -i outputfromtake.csv -o outputfromtake.json
or
./exec_genmatrix.py -i outputfromnatnet.csv -o outputfromnatnet.json


./exec_display.py -i outputfromtake.json
or
./exec_display.py -i outputfromnatnet.json


./exec_run.py -i outputfromtake.json
or
./exec_run.py -i outputfromnatnet.json
