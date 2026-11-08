/*
 * Copyright 2015 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define MODULE_TAG "mpp_bit_test"

#include <stdlib.h>
#include <string.h>

#include "mpp_log.h"
#include "mpp_common.h"
#include "mpp_bitwrite.h"

#define BIT_WRITER_BUFFER_SIZE  1024

/*
 * type is for operation type
 * 0 - plain bit put
 * 1 - bit put with detection
 * 2 - write ue
 * 3 - write se
 * 4 - align byte
 */
typedef enum BitOpsType_e {
    BIT_PUT_NO03,
    BIT_PUT,
    BIT_PUT_UE,
    BIT_PUT_SE,
    BIT_ALIGN_BYTE,
} BitOpsType;

typedef struct BitOps_t {
    BitOpsType  type;
    RK_S32      val;
    RK_S32      len;
} BitOps;

static BitOps bit_ops[] = {
    {   BIT_PUT_NO03,       0,      8,  },
    {   BIT_PUT,            0,      3,  },
    {   BIT_PUT,            0,     15,  },
    {   BIT_PUT,            0,     23,  },
    {   BIT_PUT_UE,        17,      0,  },
    {   BIT_PUT_SE,         9,      0,  },
    {   BIT_ALIGN_BYTE,     0,      0,  },
};

static MPP_RET check_writer_bounds(MppWriteCtx *writer, const RK_U8 *data,
                                   RK_U32 size, RK_U32 allocation)
{
    RK_U32 i;

    if (mpp_writer_bytes(writer) > (RK_S32)size)
        return MPP_NOK;
    for (i = size; i < allocation; i++) {
        if (data[i] != 0xa5)
            return MPP_NOK;
    }

    return MPP_OK;
}

static MPP_RET test_writer_capacity(void)
{
    RK_U8 data[8];
    RK_U8 saved[8];
    MppWriteCtx writer;
    RK_U32 size;
    RK_U32 len;
    RK_U32 offset;
    RK_U32 value;

    /* Raw fields must reserve both complete and buffered partial bytes. */
    for (size = 0; size <= 5; size++) {
        for (offset = 0; offset < 8; offset++) {
            for (len = 1; len <= 24; len++) {
                RK_U32 required = (offset + len + 7) / 8;

                memset(data, 0xa5, sizeof(data));
                mpp_writer_init(&writer, data, size);
                if (offset)
                    mpp_writer_put_raw_bits(&writer, 0, offset);
                mpp_writer_put_raw_bits(&writer, (1 << len) - 1, len);
                mpp_writer_flush(&writer);
                if (!!mpp_writer_status(&writer) != (required > size) ||
                    check_writer_bounds(&writer, data, size, sizeof(data)))
                    return MPP_NOK;

                if (required <= size) {
                    RK_U32 bit;

                    if (mpp_writer_bytes(&writer) != (RK_S32)required)
                        return MPP_NOK;
                    for (bit = 0; bit < required * 8; bit++) {
                        RK_U32 expected = bit >= offset && bit < offset + len;

                        if (((data[bit / 8] >> (7 - bit % 8)) & 1) != expected)
                            return MPP_NOK;
                    }
                } else {
                    /* Failed writes are sticky, including subsequent flush. */
                    memcpy(saved, data, sizeof(data));
                    mpp_writer_put_raw_bits(&writer, 1, 1);
                    mpp_writer_put_bits(&writer, 1, 1);
                    mpp_writer_flush(&writer);
                    if (!mpp_writer_status(&writer) ||
                        memcmp(saved, data, sizeof(data)))
                        return MPP_NOK;
                }
            }
        }
    }

    /* A prevention byte and its payload byte must fit as one operation. */
    for (size = 0; size <= 5; size++) {
        for (value = 0; value < 4; value++) {
            memset(data, 0xa5, sizeof(data));
            mpp_writer_init(&writer, data, size);
            mpp_writer_put_bits(&writer, 0, 16);
            mpp_writer_put_bits(&writer, value, 8);
            mpp_writer_flush(&writer);
            if (!!mpp_writer_status(&writer) != (size < 4) ||
                check_writer_bounds(&writer, data, size, sizeof(data)))
                return MPP_NOK;
            if (size >= 4 && (data[0] || data[1] || data[2] != 3 ||
                              data[3] != value || mpp_writer_bytes(&writer) != 4))
                return MPP_NOK;

            /* Reset clears a previous overflow and permits buffer reuse. */
            mpp_writer_reset(&writer);
            if (mpp_writer_status(&writer))
                return MPP_NOK;
            if (size) {
                mpp_writer_put_bits(&writer, 0x5a, 8);
                if (mpp_writer_status(&writer) || data[0] != 0x5a)
                    return MPP_NOK;
            }
        }
    }

    /* Exact fits are successful; flush remains repeatable and non-consuming. */
    memset(data, 0xa5, sizeof(data));
    mpp_writer_init(&writer, data, 1);
    mpp_writer_put_bits(&writer, 5, 3);
    mpp_writer_flush(&writer);
    mpp_writer_flush(&writer);
    if (data[0] != 0xa0 || writer.byte_cnt || writer.buffered_bits != 3 ||
        mpp_writer_status(&writer))
        return MPP_NOK;
    mpp_writer_align_one(&writer);
    if (data[0] != 0xbf || mpp_writer_status(&writer))
        return MPP_NOK;
    mpp_writer_trailing(&writer);
    if (!mpp_writer_status(&writer) || data[0] != 0xbf ||
        check_writer_bounds(&writer, data, 1, sizeof(data)))
        return MPP_NOK;

    mpp_writer_reset(&writer);
    mpp_writer_put_bits(&writer, 5, 3);
    mpp_writer_align_zero(&writer);
    if (data[0] != 0xa0 || mpp_writer_status(&writer))
        return MPP_NOK;

    mpp_log("bit writer capacity tests passed\n");
    return MPP_OK;
}

void proc_bit_ops(MppWriteCtx *writer, BitOps *ops)
{
    switch (ops->type) {
    case BIT_PUT_NO03 : {
        mpp_writer_put_raw_bits(writer, ops->val, ops->len);
    } break;
    case BIT_PUT : {
        mpp_writer_put_bits(writer, ops->val, ops->len);
    } break;
    case BIT_PUT_UE : {
        mpp_writer_put_ue(writer, ops->val);
    } break;
    case BIT_PUT_SE : {
        mpp_writer_put_ue(writer, ops->val);
    } break;
    case BIT_ALIGN_BYTE : {
        mpp_writer_trailing(writer);
    } break;
    default : {
        mpp_err("invalid ops type %d\n", ops->type);
    } break;
    }
}

int main()
{
    MPP_RET ret = MPP_ERR_UNKNOW;
    void *data = NULL;
    size_t size = BIT_WRITER_BUFFER_SIZE;
    MppWriteCtx writer;
    RK_U32 len_byte;
    RK_U32 i;
    RK_U32 buf_len = 0;
    char buf[BIT_WRITER_BUFFER_SIZE];

    mpp_log("mpp_bit_test start\n");

    ret = test_writer_capacity();
    if (ret)
        goto TEST_FAILED;

    data = malloc(size);
    if (NULL == data) {
        mpp_err("mpp_bit_test malloc failed\n");
        ret = MPP_ERR_MALLOC;
        goto TEST_FAILED;
    }

    mpp_writer_init(&writer, data, size);

    for (i = 0; i < MPP_ARRAY_ELEMS(bit_ops); i++)
        proc_bit_ops(&writer, &bit_ops[i]);

    len_byte = writer.byte_cnt;

    for (i = 0; i < len_byte; i++) {
        buf_len += snprintf(buf + buf_len, sizeof(buf) - buf_len,
                            "%02x ", writer.buffer[i]);
    }

    mpp_log("stream %s\n", buf);

    ret = MPP_OK;
TEST_FAILED:
    if (data)
        free(data);

    if (ret)
        mpp_log("mpp_bit_test failed\n");
    else
        mpp_log("mpp_bit_test success\n");

    return ret;
}

