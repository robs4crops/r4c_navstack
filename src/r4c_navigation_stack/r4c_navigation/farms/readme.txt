The geojson files shared with with the farming controller (FC), from LMS are:

1. bounding_box.geojson
2. entry_point.geojson
3. exit_point.geojson
4. headlands.geojson
5. working_lanes.geojson
6. zones.geojson

These geojson files are stored along with the farm.json file, for each pilot, for reference, in case we have to upload them to the FC at any point in time.

URL for the fields in the FC: http://10.147.17.52:3000/fields

The process to create the geojson files is the following:
1. In the field you measure the points of interest using the GNSS receiver from the AGCBox, for the structure you want to register: bounding box, entry point, exit point, a headland, a working lane, or special zone (like a warehouse, or a fuel station, etc.).
2. You create the proper geojson file with the points registered in step 1.
3. Upload the created geojson file to the proper field in the FC's field page.

The farms.json file is created base on the data found in the files headlands.geojson and working_lanes.geojson.
