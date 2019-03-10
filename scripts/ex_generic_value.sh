./generate_itoms.py
./fault_injection.py -t  1.5  3.5 -s     generated.obj /a/data
./fault_injection.py -t  5.5  7.5 -n 1   generated.obj /c/data
./run.py -m ../config/generic.pl -v x -u ../config/generic.yaml -p 1e9 generated.obj
