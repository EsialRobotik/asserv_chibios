#ifndef CONFIGURATION_REPRESENTATION_H_
#define CONFIGURATION_REPRESENTATION_H_

#include "ConfigurationInterface.h"

class ConfigurationRepresentation
{
public:
	explicit ConfigurationRepresentation(Configuration *angle_regulator, Configuration *dist_regulator,
										 Configuration *angle_acc, Configuration *dist_acc,
										 Configuration *speed_right, Configuration *speed_left);

	void generateRepresentation(QCBOREncodeContext &EncodeCtx);

	void applyConfiguration(QCBORDecodeContext &decodeCtx);

private:

	Configuration * m_angle_regulator;
	Configuration * m_dist_regulator;
	Configuration * m_angle_acc;
	Configuration * m_dist_acc;
	Configuration * m_speed_right;
	Configuration * m_speed_left;
};

#endif /* CONFIGURATION_REPRESENTATION_H_ */
