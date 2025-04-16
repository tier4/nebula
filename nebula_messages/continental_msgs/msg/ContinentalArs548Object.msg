uint32 object_id
uint16 age
uint8 status_measurement
uint8 status_movement
uint8 position_reference

geometry_msgs/Point position
geometry_msgs/Vector3 position_std
float32 position_covariance_xy
float32 orientation
float32 orientation_std

float32 raw_existence_probability # 0 ~ 100

uint8 raw_classification_car # 0 ~ 100
uint8 raw_classification_truck # 0 ~ 100
uint8 raw_classification_motorcycle # 0 ~ 100
uint8 raw_classification_bicycle # 0 ~ 100
uint8 raw_classification_pedestrian # 0 ~ 100
uint8 raw_classification_animal # 0 ~ 100
uint8 raw_classification_hazard # 0 ~ 100
uint8 raw_classification_unknown # 0 ~ 100

geometry_msgs/Vector3 absolute_velocity
geometry_msgs/Vector3 absolute_velocity_std
float32 absolute_velocity_covariance_xy

geometry_msgs/Vector3 relative_velocity
geometry_msgs/Vector3 relative_velocity_std
float32 relative_velocity_covariance_xy

geometry_msgs/Vector3 absolute_acceleration
geometry_msgs/Vector3 absolute_acceleration_std
float32 absolute_acceleration_covariance_xy

geometry_msgs/Vector3 relative_acceleration
geometry_msgs/Vector3 relative_acceleration_std
float32 relative_acceleration_covariance_xy

float32 orientation_rate_mean
float32 orientation_rate_std

float32 shape_length_edge_mean
float32 shape_width_edge_mean
