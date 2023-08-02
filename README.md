# Johnny Cache: the End of DRAM Cache Conflicts (in Tiered Main Memory Systems)

Paper: [https://www.usenix.org/conference/osdi23/presentation/lepers](https://www.usenix.org/conference/osdi23/presentation/lepers)

## Usage
Patch your kernel using the provided patches.

Edit jc.c to configure JC (see comment on top of the file). I recommend to first try to module without the dynamic policy (default setting).

JC only handles 2MB pages. To force the application to allocate huge pages, we use TCMALLOC ([https://github.com/gperftools/gperftools](https://github.com/gperftools/gperftools)).

```bash
# Reserve a few large pages
sudo sh -c "echo 174080 > /sys/devices/system/node/node0/hugepages/hugepages-2048kB/nr_hugepages" # 340GB of large pages
sudo mount -t hugetlbfs none /mnt/hugetlbfs
sudo chmod 777 /mnt/hugetlbfs/

# Insert the module and launch BC with 20 threads
sudo dmesg -c; # clear dmesg
make; # READ the comment of jc.c BEFORE compiling!
sudo insmod jc.ko;

numactl --cpunodebind 0 env OMP_NUM_THREADS=20 TCMALLOC_MEMFS_MALLOC_PATH=/mnt/hugetlbfs/ LD_PRELOAD=~/gperftools/.libs/libtcmalloc.so ~/hemem/apps/gapbs/bc -u 29;

sudo rmmod jc;
sudo dmesg -c; # print stats
```

