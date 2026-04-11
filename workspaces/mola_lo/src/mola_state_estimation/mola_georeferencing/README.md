# mola_georeferencing

## Georeferencing a map

```bash
# Create the mm:
sm2mm -i INPUT_WITH_GPS.simplemap \
 -o MAP.mm \
 -p pipeline.yaml

# georeference it:
mola-sm-georeferencing -i INPUT_WITH_GPS.simplemap --write-into MAP.mm
```

# License
Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).
