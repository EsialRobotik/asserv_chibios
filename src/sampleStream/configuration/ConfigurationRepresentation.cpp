#include "ConfigurationRepresentation.h"

ConfigurationRepresentation::ConfigurationRepresentation(
		Configuration *angle_regulator, Configuration *dist_regulator,
		Configuration *angle_acc, Configuration *dist_acc,
		Configuration *speed_right, Configuration *speed_left)
{
	m_angle_regulator = angle_regulator;
	m_dist_regulator = dist_regulator;
	m_angle_acc = angle_acc;
	m_dist_acc = dist_acc;
	m_speed_right = speed_right;
	m_speed_left = speed_left;
}

void ConfigurationRepresentation::generateRepresentation(Cbore & cbor_representation)
{
	cbor_representation = m_dist_regulator->getConfiguration( cbor_representation.map().key("dist_regulator") );
	cbor_representation = m_angle_regulator->getConfiguration( cbor_representation.key("angle_regulator") );

	cbor_representation = m_dist_acc->getConfiguration( cbor_representation.key("dist_acc") );
	cbor_representation = m_angle_acc->getConfiguration( cbor_representation.key("angle_acc") );

	cbor_representation = m_speed_right->getConfiguration( cbor_representation.key("speed_right"));
	cbor_representation = m_speed_left->getConfiguration( cbor_representation.key("speed_left"));
	cbor_representation.end();
}
