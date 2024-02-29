talker_pids=$(ps -a -T | grep talker)
# echo ${talker_pids[1]}



# for i in "${talker_pids[@]}"; do
#   echo "$i"
# done

arr=($talker_pids)

for i in "${arr[@]}"; do
  echo "$i"
#   echo ...
done