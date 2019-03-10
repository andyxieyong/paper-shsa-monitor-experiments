./generate_itoms.py
./fault_injection.py -t  1.5  2.5 -s      generated.obj /b/data
./fault_injection.py -t  1.5  2.5 -d 0.6  generated.obj /a/data
./fault_injection.py -t  4.5  5.5 -s      generated.obj /a/data
./fault_injection.py -t  4.5  5.5 -d 0.5  generated.obj /a/data
./run.py -m ../config/generic.pl -v x -u ../config/generic.yaml -p 1e9 -b 1 -o run_gt1.obj generated.obj
./run.py -m ../config/generic.pl -v x -u ../config/generic.yaml -p 1e9 -b 2 -o run_gt2.obj generated.obj
