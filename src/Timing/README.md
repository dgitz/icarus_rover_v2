Author: David Gitz
Task: Timing
Nodes:
A. timemaster_node
Usage: This node should be run on 1 device.
Purpose:
1. Reads a time source on the /1PPS topic (or generates if an external source is not available)
2. Publishes to the following topics:
 /1PPS @ 1000 mS (if not already published by an external source)

B. timeslave_node
Usage: This node should be run on each device
Purpose:
1. Uses NTP to check time delays between each Device and the device acting as ROSCore
