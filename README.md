Bullet physics library ported for Cheerp
========================================

Please report bugs on launchpad:
https://bugs.launchpad.net/cheerp

Installation
------------

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/opt/cheerp/share/cmake/Modules/CheerpToolchain.cmake -DBUILD_EXTRAS=OFF -DCMAKE_INSTALL_PREFIX=/opt/cheerp
make
nodejs Demos/HelloWorld/AppHelloWorld.js
```

