#!bin/bash

python ppo.py --capacity 100 --seed 1 
python ppo.py --capacity 100 --seed 2
python ppo.py --capacity 100 --seed 55
python ppo.py --capacity 100 --seed 77

# # python ppo.py --apply_genetic_ops 1 --seed 1
# # python ppo.py --apply_genetic_ops 1 --seed 2
# # python ppo.py --apply_genetic_ops 1 --seed 55
# # python ppo.py --apply_genetic_ops 1 --seed 77

# python ppo.py --nk_buffer 1 --apply_genetic_ops 1 --seed 1
# python ppo.py --nk_buffer 1 --apply_genetic_ops 1 --seed 2
# python ppo.py --nk_buffer 1 --apply_genetic_ops 1 --seed 55
# python ppo.py --nk_buffer 1 --apply_genetic_ops 1 --seed 77
