#ifndef OV2640_REG_H
#define OV2640_REG_H

#include <cmsis_compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OV2640_ID         0x2642U

#define OV2640_REG_MANUF  0x0AU
#define OV2640_REG_PIDH   0x0AU
#define OV2640_REG_PIDL   0x0BU

#define OV2640_REG_PAGE   0xFFU

#define OV2640_REG_COM7   0x12U
#define OV2640_COM7_SRST  0x80U

#define OV2640_REG_CLKRC  0x11U

typedef int32_t (*OV2640_Write_Func)(void *, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*OV2640_Read_Func)(void *, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*OV2640_Modify_Func)(void *, uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  OV2640_Write_Func   WriteReg;
  OV2640_Read_Func    ReadReg;
  OV2640_Modify_Func  ModifyReg;
  void                *handle;
} ov2640_ctx_t;

int32_t ov2640_write_reg(ov2640_ctx_t *ctx, uint16_t reg, uint8_t *pdata, uint16_t length);
int32_t ov2640_read_reg(ov2640_ctx_t *ctx, uint16_t reg, uint8_t *pdata, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
