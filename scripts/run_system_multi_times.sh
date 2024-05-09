### check input validity
if [ $# -eq 0 ]; then
    echo "No arguments provided. A scheduler is needed: CFS, RM, optimizerBF, optimizerIncremental"
    echo "Exiting the script ..."
    exit 0
fi
if [ $1 != "RM" ] && [ $1 != "CFS" ] && [ $1 != "optimizerBF" ] && [ $1 != "optimizerIncremental" ] ; then
    echo "Scheduler $1 is not supported."
    echo "Supported schedulers are: CFS, RM, optimizerBF, optimizerIncremental"
    echo "Exiting the script ..."
    exit 0
fi

num_iterations=10

for ((i=1; i<=$num_iterations; i++))
do
    ./start_system.sh $1
    sleep 60 # sleep 60 seconds
done

echo "Loop finished"