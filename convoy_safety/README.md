# Convoy safety

This package contains nodes to make sure the vehicles are behaving safely.

## Nodes

### autonomous_control_node

Makes sure the vehicle only publishes to the true drive topic if a human 
supervisor is holding down the `R1` button on the teleop controller.
