./getbuildings.py -i 'Take 2022-11-08 08.26.58 AM.csv' -o outputfromtake.csv
or
./getbuildings.py -o outputfromnatnet.csv


./genmatrix.py -i outputfromtake.csv -o outputfromtake.json
or
./genmatrix.py -i outputfromnatnet.csv -o outputfromnatnet.json


./display.py -i outputfromtake.json
or
./display.py -i outputfromnatnet.json


./run.py -i outputfromtake.json
or
./run.py -i outputfromnatnet.json


