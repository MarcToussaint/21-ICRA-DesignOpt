
para=5
numCases=10

rm -Rf dat.*

task(){
    path="dat.$mode.$seed"
    mkdir -p "$path"
    cmd="time ./x.exe -disableGui -scenario resorting -mode $1 -seed $2 -numCases $numCases > $path/output"
    echo "$cmd"
    eval "$cmd"
#    sleep 0.5
}

(
    for mode in 3 2 1 0; do
	for seed in 1 2 3 4 5 6 7 8 9 10; do
	    ((i=i%para)); ((i++==0)) && wait
	    task "$mode" "$seed" &
	done
    done
    wait
)

#./x.exe -disableGui -seed 0 -mode 2 -numCases 2 &

cmd="zip -r dat-`date +%y-%m-%d-%H-%M-%S`.zip dat.*"
echo "$cmd"
eval "$cmd"
