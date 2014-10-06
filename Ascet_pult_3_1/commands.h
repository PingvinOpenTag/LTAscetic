enum argument_error {
ok,						//аргумент имеет корректное значение
invalid,				//недопустимый аргумент   
out_of_range, 			//аргумент выходит за пределы корректных значений
empty					//аргумент пустой
};
typedef enum argument_error TARGUMENT_ERROR;



const unsigned char command_0[] PROGMEM = "bullets_in_clip=";  // 
const unsigned char command_1[] PROGMEM = "bullets_in_clip?";  // 
const unsigned char command_2[] PROGMEM = "protocol?";
const unsigned char command_3[] PROGMEM = "clips=";  // 
const unsigned char command_4[] PROGMEM = "clips?";  // 
const unsigned char command_5[] PROGMEM = "prepare_to_write_block";
const unsigned char command_6[] PROGMEM = "read_block";
const unsigned char command_7[] PROGMEM = "sound_1_adress=";
const unsigned char command_8[] PROGMEM = "sound_1_adress?";
const unsigned char command_9[] PROGMEM = "sound_1_size=";
const unsigned char command_10[] PROGMEM = "sound_1_size?";
const unsigned char command_11[] PROGMEM = "sound_2_adress=";
const unsigned char command_12[] PROGMEM = "sound_2_adress?";
const unsigned char command_13[] PROGMEM = "sound_2_size=";
const unsigned char command_14[] PROGMEM = "sound_2_size?";
const unsigned char command_15[] PROGMEM = "sound_3_adress=";
const unsigned char command_16[] PROGMEM = "sound_3_adress?";
const unsigned char command_17[] PROGMEM = "sound_3_size=";
const unsigned char command_18[] PROGMEM = "sound_3_size?";
const unsigned char command_19[] PROGMEM = "sound_4_adress=";
const unsigned char command_20[] PROGMEM = "sound_4_adress?";
const unsigned char command_21[] PROGMEM = "sound_4_size=";
const unsigned char command_22[] PROGMEM = "sound_4_size?";
const unsigned char command_23[] PROGMEM = "sound_5_adress=";
const unsigned char command_24[] PROGMEM = "sound_5_adress?";
const unsigned char command_25[] PROGMEM = "sound_5_size=";
const unsigned char command_26[] PROGMEM = "sound_5_size?";
const unsigned char command_27[] PROGMEM = "sound_6_adress=";
const unsigned char command_28[] PROGMEM = "sound_6_adress?";
const unsigned char command_29[] PROGMEM = "sound_6_size=";
const unsigned char command_30[] PROGMEM = "sound_6_size?";
const unsigned char command_31[] PROGMEM = "play_sound";
const unsigned char command_32[] PROGMEM = "play_shot_sound";
const unsigned char command_33[] PROGMEM = "player_id=";
const unsigned char command_34[] PROGMEM = "player_id?";
const unsigned char command_35[] PROGMEM = "damage_index=";
const unsigned char command_36[] PROGMEM = "damage_index?";
const unsigned char command_37[] PROGMEM = "ir_power=";
const unsigned char command_38[] PROGMEM = "ir_power?";
const unsigned char command_39[] PROGMEM = "friendly_fire=";
const unsigned char command_40[] PROGMEM = "friendly_fire?";
const unsigned char command_41[] PROGMEM = "team_id=";
const unsigned char command_42[] PROGMEM = "team_id?";
const unsigned char command_43[] PROGMEM = "batt_full_voltage=";
const unsigned char command_44[] PROGMEM = "batt_full_voltage?";
const unsigned char command_45[] PROGMEM = "batt_low_voltage=";
const unsigned char command_46[] PROGMEM = "batt_low_voltage?";
const unsigned char command_47[] PROGMEM = "pult_key_up_cmd=";
const unsigned char command_48[] PROGMEM = "pult_key_up_cmd?";
const unsigned char command_49[] PROGMEM = "pult_key_right_cmd=";
const unsigned char command_50[] PROGMEM = "pult_key_right_cmd?";
const unsigned char command_51[] PROGMEM = "pult_key_down_cmd=";
const unsigned char command_52[] PROGMEM = "pult_key_down_cmd?";
const unsigned char command_53[] PROGMEM = "pult_key_left_cmd=";
const unsigned char command_54[] PROGMEM = "pult_key_left_cmd?";
const unsigned char command_55[] PROGMEM = "pult_key_central_cmd=";
const unsigned char command_56[] PROGMEM = "pult_key_central_cmd?";
const unsigned char command_57[] PROGMEM = "pult_shift_and_key_up_cmd=";
const unsigned char command_58[] PROGMEM = "pult_shift_and_key_up_cmd?";
const unsigned char command_59[] PROGMEM = "pult_shift_and_key_right_cmd=";
const unsigned char command_60[] PROGMEM = "pult_shift_and_key_right_cmd?";
const unsigned char command_61[] PROGMEM = "pult_shift_and_key_down_cmd=";
const unsigned char command_62[] PROGMEM = "pult_shift_and_key_down_cmd?";
const unsigned char command_63[] PROGMEM = "pult_shift_and_key_left_cmd=";
const unsigned char command_64[] PROGMEM = "pult_shift_and_key_left_cmd?";
const unsigned char command_65[] PROGMEM = "pult_shift_and_central_cmd=";
const unsigned char command_66[] PROGMEM = "pult_shift_and_central_cmd?";
const unsigned char command_67[] PROGMEM = "pult_reload_cmd=";
const unsigned char command_68[] PROGMEM = "pult_reload_cmd?";







const unsigned char* commandsPointers[] PROGMEM = 
{command_0,
command_1,
command_2,
command_3,
command_4,
command_5,
command_6,
command_7,
command_8,
command_9,
command_10,
command_11,
command_12,
command_13,
command_14,
command_15,
command_16,
command_17,
command_18,
command_19,
command_20,
command_21,
command_22,
command_23,
command_24,
command_25,
command_26,
command_27,
command_28,
command_29,
command_30,
command_31,
command_32,
command_33,
command_34,
command_35,
command_36,
command_37,
command_38,
command_39,
command_40,
command_41,
command_42,
command_43,
command_44,
command_45,
command_46,
command_47,
command_48,
command_49,
command_50,
command_51,
command_52,
command_53,
command_54,
command_55,
command_56,
command_57,
command_58,
command_59,
command_60,
command_61,
command_62,
command_63,
command_64,
command_65,
command_66,
command_67,
command_68
};



const unsigned char unknown_command_error[] PROGMEM = "ERROR:unknown command\r\n";
const unsigned char command_error[] PROGMEM = "ERROR\r\n";
const unsigned char parameter_empty_error[] PROGMEM = "ERROR:parameter is not specified\r\n";
const unsigned char parameter_out_of_range_error[] PROGMEM = "ERROR:parameter out of range\r\n";
const unsigned char parameter_invalid_error[] PROGMEM = "ERROR:invalid parameter\r\n";
const unsigned char ok_string[] PROGMEM = "\r\nOK\r\n";



#define MAX_BULL_IN_CLIP 90
#define MAX_BULL_IN_CLIP_STR '90'

const unsigned char protocol[] PROGMEM = 	//"bullets_in_clip;int(0,90);bullets_in_clip?;bullets_in_clip=\r\n"
											"Количество патронов в магазине;int(0,90);bullets_in_clip?;bullets_in_clip=\r\n"
											"Магазинов;int(0,100);clips?;clips=\r\n"
											"Идентификатор игрока;int(0,127);player_id?;player_id=\r\n"
											"Цвет команды;enum(Красная,Синяя,Желтая,Зеленая);team_id?;team_id=\r\n"
											"Наносимый урон;enum(1%,2%,4%,5%,7%,10%,15%,17%,20%,25%,30%,35%,40%,50%,75%,100%);damage_index?;damage_index=\r\n"
											"Дружественный огонь;enum(Нет,Да);friendly_fire?;friendly_fire=\r\n"
											"Мощность ИК излучения;enum(Для игры в помещении,Для игры на улице);ir_power?;ir_power=\r\n"
											"Напряжение заряж. батареи мВ;int(5000,45000);batt_full_voltage?;batt_full_voltage=\r\n"
											"Напряжение разряж. батареи мВ;int(4500,45000);batt_low_voltage?;batt_low_voltage=\r\n";



