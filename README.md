Bullet physics library ported for Duetto
========================================

Please report bugs on launchpad:
https://bugs.launchpad.net/duetto

Installation
------------

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/opt/duetto/share/cmake/Modules/DuettoToolchain.cmake -DBUILD_EXTRAS=OFF -DCMAKE_INSTALL_PREFIX=/opt/duetto
make
nodejs Demos/HelloWorld/AppHelloWorld.js
```

