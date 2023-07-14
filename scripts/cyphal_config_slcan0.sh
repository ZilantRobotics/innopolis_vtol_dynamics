#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CYPHAL_PKG_DIR=$SCRIPT_DIR/../communicators/cyphal_communicator

# specify your pathes here:
export YAKUT_COMPILE_OUTPUT=$CYPHAL_PKG_DIR/compile_output
REG_DATA_TYPE_PATH_UAVCAN=$CYPHAL_PKG_DIR/public_regulated_data_types/uavcan
REG_DATA_TYPE_PATH_REG=$CYPHAL_PKG_DIR/public_regulated_data_types/reg

export YAKUT_PATH="$YAKUT_COMPILE_OUTPUT"
export REG_DATA_TYPE_PATH="$REG_DATA_TYPE_PATH_UAVCAN $REG_DATA_TYPE_PATH_REG"

export UAVCAN__CAN__IFACE='socketcan:slcan0'
export UAVCAN__CAN__MTU=8
export UAVCAN__NODE__ID=2

export ALLOCATION_TABLE_PATH=allocation_table.db

source $SCRIPT_DIR/cyphal_port_id.sh
