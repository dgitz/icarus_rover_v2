Author: David Gitz
Task: Sandbox-Vision
Purpose: All Vision-related Nodes

Nodes:
A. cameracapture_node
Purpose:
Mode: "capture"
1. Read Camera at specified Device (Optional, in case the image is being published from another node)
2. Do Edge Detection on defined topic (such as <camera>/image) and output to  <camera>/image_edge
Mode: "resample"
1. Reads image topic and rescales to match configured size.  Supports encoding: rgb8, 32FC1

B. tracking_node
Purpose:
1. Reads target images stored in defined path.
2. Reads image at defined topic.
3. Outputs position of found target in image to: 

C. vision_node
Purpose: Obsolete node, keeping for archives and possible future expansion.

C. cameramanager_node
Purpose: 
1. Publishes tf info for cameras (In Work)
2. Publishes diagnostic, heartbeat and resource information for specified external tasks.
