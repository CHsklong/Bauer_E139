
#include "work_io.h"
#include "mcdrv.h"

IN_t input_info[INPUT_IDX_MAX] = {
	/***    idx   	  				   use   type  def_state filter alm_edge	***/
	{IN_INPUT1, 1, 0, 1, 3, 0},
	{IN_INPUT2, 1, 0, 1, 3, 0},
	{IN_INPUT3, 1, 0, 1, 3, 0},

};
OUT_t output_info[OUTPUT_IDX_MAX] = {
	{OUTPUT_IDX1, 0}, //
};

uint8_t input_state[INPUT_IDX_MAX];		   //
uint8_t input_filter_count[INPUT_IDX_MAX]; //
IN_EDGE_t input_edge[INPUT_IDX_MAX];	   //
uint8_t input_stable[INPUT_IDX_MAX];	   //

IO_Fun_Tag io_fun_st;
uint8_t hal_input_check(input_idx_t idx)
{
	uint8_t ret = IO_STAT_INVAILD;
	switch (idx)
	{
	case IN_INPUT1: //
		ret = BOARD_Input(IN_INPUT1);
		break;
	case IN_INPUT2: //
		ret = BOARD_Input(IN_INPUT2);
		break;
	case IN_INPUT3: //
		ret = BOARD_Input(IN_INPUT3);
		break;
        case INPUT_IDX_MAX:
                break;
	}
	return ret;
}
/*  */
void work_input_scan(void) //
{
	int idx;
	uint8_t val;
	IN_t *inp;

	for (idx = 0; idx < INPUT_IDX_MAX; idx++)
	{
		inp = input_info + idx;
		if ((inp->use > 0))
		{
			val = hal_input_check((input_idx_t)idx);
			if (val == IO_STAT_INVAILD)
				continue;
			val ^= inp->def_state;
			/*  --  */
			if (input_state[idx] != val)
			{

				if ((++input_filter_count[idx]) >= inp->filter)
				{
					input_filter_count[idx] = 0;
					input_state[idx] = val;
					input_stable[idx] = val ^ inp->def_state;
					if (val == 0)
					{
						input_edge[idx] = EDGE_FALLING; /*  */
					}
					else
					{
						input_edge[idx] = EDGE_RISING; /*  */
					}
				}
			}
			else
			{
				input_filter_count[idx] = 0;

				input_edge[idx] = EDGE_NORMAL;
			}
		}
	}
}

/*  */
uint8_t work_input_chk_port(input_idx_t idx)
{
	IN_t *inp;
	uint8_t val = 0;

	if (idx < INPUT_IDX_MAX)
	{
		inp = input_info + idx;

		/*  */
		if ((inp->use > 0))
		{
			/*  */
			val = hal_input_check(idx);
			val ^= inp->def_state;
		}
	}
	return (val);
}

/*  */
uint8_t work_input_get_state(input_idx_t idx)
{
	if (idx < INPUT_IDX_MAX)
	{
		return (input_state[idx]);
	}
	return (0);
}
/*  */
IN_EDGE_t work_input_get_edge(input_idx_t idx)
{
	if (idx < INPUT_IDX_MAX)
	{
		return (input_edge[idx]);
	}
	return (0);
}

/*  */
uint8_t work_input_get_stable(input_idx_t idx)
{
	if (idx < INPUT_IDX_MAX)
	{
		return (input_stable[idx]);
	}
	return (0);
}

/*  */
IN_t *work_input_get_info(input_idx_t idx)
{
	if (idx >= INPUT_IDX_MAX)
		return (0);
	return (input_info + idx);
}
void work_output(output_idx_t idx, PIN_STAT stat)
{
	uint8_t out = (output_info[idx].def_state ^ stat) & 0x01;
	output_info[idx].out_state = stat;
	switch (idx)
	{
	case OUTPUT_IDX1:
		BOARD_Output(out);
		break;
        case OUTPUT_IDX_MAX:
                break;
	}
}

void work_input_init(void)
{
	uint16_t i;
	//int err = 0;
	//IN_t *in;
	/*  */
	for (i = 0; i < INPUT_IDX_MAX; i++)
	{
		//in = input_info + i;
		/*  */
		input_filter_count[i] = 0;
		// input_state[i] = in->def_state;
		input_state[i] = 0;
	}
}
void work_input_set_def_state(input_idx_t idx,uint8_t def_state)
{
  if (idx < INPUT_IDX_MAX)
  {
     input_info[idx].def_state = def_state ;
  }
  
}
void work_io_set_fun(int32_t io_fun)
{
	io_fun_st.io_fun = io_fun;
}

IO_Fun_Tag work_io_get_fun(void)
{
	return io_fun_st;
}
