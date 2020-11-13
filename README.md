# 21-ICRA-DesignOpt

supplementary material to an ICRA submission

## Quick Start

```
mkdir -p $HOME/git
cd $HOME/git
git clone https://github.com/MarcToussaint/21-ICRA-DesignOpt.git
cd 21-ICRA-DesignOpt

git submodule init
git submodule update

make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
cd demo
make -j $(command nproc)
./x.exe -seed 0 -mode 1 -numCases 2 -scenario wrenchTool
```

(In `demo`, see also the rai.cfg and eval-run.sh.)
