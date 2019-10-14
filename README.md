# Blobs

## Library issues:

For file `Adafruit_SharpMem.h`, there seems to be some linking issue. One solution to this it to simply apply the following patch manually:

```
// Comment out this line.
//#include <Adafruit GFX Library_ID13/Adafruit_GFX.h>

// Add this line in its place.
#include <../Adafruit GFX Library_ID13/Adafruit_GFX.h>
```