#!/usr/bin/env python3
"""Convert a Natural Earth coastline GeoJSON into the raylib build's coastline file.

Natural Earth (https://www.naturalearthdata.com) is public domain. The bundled
file came from the 50m set:

    curl -LO https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_50m_coastline.geojson
    python3 tools/coastline_to_dat.py ne_50m_coastline.geojson src/renderer/assets/coastline_50m.dat

Output layout, little-endian:
    char[8]  "ACECOAST"
    uint32   version (1)
    uint32   polyline count
    per polyline: uint32 point count, then that many (float32 lon_deg, float32 lat_deg)
"""
import json
import struct
import sys


def polylines(geojson):
    for feature in geojson["features"]:
        geom = feature["geometry"]
        if geom["type"] == "LineString":
            yield geom["coordinates"]
        elif geom["type"] == "MultiLineString":
            yield from geom["coordinates"]


def main(src, dst):
    with open(src) as f:
        lines = [l for l in polylines(json.load(f)) if len(l) >= 2]
    with open(dst, "wb") as out:
        out.write(b"ACECOAST")
        out.write(struct.pack("<II", 1, len(lines)))
        for line in lines:
            out.write(struct.pack("<I", len(line)))
            for lon, lat, *_ in line:
                out.write(struct.pack("<ff", lon, lat))
    print(f"{dst}: {len(lines)} polylines, {sum(len(l) for l in lines)} points")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: coastline_to_dat.py <input.geojson> <output.dat>")
    main(sys.argv[1], sys.argv[2])
