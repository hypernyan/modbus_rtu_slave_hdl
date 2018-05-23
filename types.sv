package types;
typedef enum logic [13:0] {
	RX_IDLE_S,
	RX_FUNC_CODE_S,
	RX_START_ADDR_W_HI_S, 
	RX_START_ADDR_W_LO_S,
	RX_START_ADDR_R_HI_S, 
	RX_START_ADDR_R_LO_S,
	RX_QUANTITY_W_HI_S,
	RX_QUANTITY_W_LO_S,
	RX_QUANTITY_R_HI_S,
	RX_QUANTITY_R_LO_S,
	RX_BYTE_COUNT_S,
	RX_DATA_S,
	RX_CRC_HI_S,
	RX_CRC_LO_S
} modbus_fsm_rx_t;

typedef enum logic [10:0] {
	TX_IDLE_S,
	TX_FUNC_CODE_S,
	TX_EX_CODE_S,
	TX_START_ADDR_HI_S,
	TX_START_ADDR_LO_S,
	TX_QUANTITY_HI_S,
	TX_QUANTITY_LO_S,
	TX_BYTE_COUNT_S,
	TX_DATA_S,
	TX_CRC_HI_S,
	TX_CRC_LO_S
} modbus_fsm_tx_t;

typedef enum logic [2:0]{
	MEM_IDLE_S,
	WRITE_REG_S,
	EX_SEND_S
} mem_fsm_t;

typedef enum logic [7:0] {
// COILS
	READ_COILS                    = 8'h01,
	WRITE_SINGLE_COIL             = 8'h05,
	WRITE_MULTIPLE_COILS          = 8'h0f,
// HOLDING REGISTERS	
	READ_HOLDING_REGISTERS        = 8'h03,
// DISCRETE INPUTS
	READ_DISCRETE_INPUTS          = 8'h02,
// INPUT REGISTERS
	READ_INPUT_REGISTERS          = 8'h04, 
// INPUT OT HOLDING REGISTERS
	WRITE_SINGLE_REGISTER         = 8'h06,
	WRITE_MULTIPLE_REGISTERS      = 8'h10,
	READ_WRITE_MULTIPLE_REGISTERS = 8'h17,
// OTHERS
	READ_EXCEPTION_STATUS         = 8'h07
} func_code_t;

typedef enum logic [7:0] {
	BAD_CRC,
	UNEXPECTED_PACKET_END,
	BAD_FUNC_CODE
} ex_code_t;

typedef enum logic {
	rx = 1'b0,
	tx = 1'b1
} crc_master_t;

typedef enum logic [2:0] {
	CRC_INIT_S,
	CRC_XOR_SHIFT_S,
	CRC_LSB_S,
	CRC_WAIT_S
} CRC_FSM_t;

typedef struct {
	func_code_t func_code;
	ex_code_t   ex_code;
	bit [15:0]  quantity_w;
	bit [15:0]  quantity_r;
	bit [7:0]   byte_count;
	bit [15:0]  start_addr_w;
	bit [15:0]  start_addr_r;
} modbus_packet_t;

endpackage
