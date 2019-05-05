/*
 * IIOProvider.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#ifndef SRC_IIOPROVIDER_H_
#define SRC_IIOPROVIDER_H_

#include <stdint.h>

class IIOProvider {
public:
    IIOProvider() {}
    virtual bool   IsConnected() = 0;
    virtual double GetByteCount() = 0;
    virtual uint8_t GetUpdateCount() = 0;
    virtual void   SetUpdateRateHz(uint8_t update_rate) = 0;
    virtual void   ZeroYaw() = 0;
    virtual void   ZeroDisplacement() = 0;
    virtual void   Run() = 0;
    virtual void   Stop() = 0;
    virtual void 	ResetConnection() = 0;
    virtual bool IsResetting() = 0;
};

#endif /* SRC_IIOPROVIDER_H_ */
