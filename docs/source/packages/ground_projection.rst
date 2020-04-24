ground\_projection package
==========================

.. contents::

The ``ground_projectino`` package provides the tools for projecting line segments from an image reference frame to the ground reference frame, as well as a ROS node that implements that. It has been designed to be a part of the lane localization pipeline. Consists of the ROS node :py:class:`nodes.GroundProjectionNode` and the :py:mod:`ground_projection` library.


GroundProjectionNode
--------------------

.. autoclass:: nodes.GroundProjection

Included libraries
------------------

.. automodule:: ground_projection