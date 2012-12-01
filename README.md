interactive_marker_proxy
========================

ROS node that acts as a proxy server for Interactive Markers, caring about message demultiplexing and tf transforms.

How to use
----------

To run a proxy for the Interactive Markers on the topic `/basic_controls` and transform
everything into the frame `/base_link`, use

```rosrun interactive_marker_proxy proxy  topic_ns:=/basic_controls target_frame:=/base_link```