# Blobs

## Library issues:

Unfortunately, lack of time prevents me from find proper solutions for these issues.
For file `Adafruit_SharpMem.h`, there seems to be some linking issue. One solution to this it to simply apply the following patch manually:

```
// Comment out this line.
//#include <Adafruit_GFX.h>

// Add this line in its place.
#include <../Adafruit GFX Library_ID13/Adafruit_GFX.h>
```

For file `b2Controller.h`, apply:

```
// Comment out these lines.
//#include "../../Dynamics/b2World.h"
//#include "../../Dynamics/b2Body.h"

// Add these lines.
#include "../Box2D/Box2D/Dynamics/b2World.h"
#include "../Box2D/Box2D/Dynamics/b2Body.h"
```


For file `Stdafx.h`, apply:

```
// Comment out this line:
//#include "Box2D.h"
```

For file `b2BuoyancyController.cpp`, apply:

```
//#include "../b2Fixture.h"
#include "../Box2D/Box2D/Dynamics/b2Fixture.h"
```

For Box2D, delete `Contributors`, `TestBeds`