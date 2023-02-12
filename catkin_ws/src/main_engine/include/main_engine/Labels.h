#ifndef LABELS_H_
#define LABELS_H_

#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;

namespace MSG {
  const std::string ARE_YOU_READY    = "Are_you_ready?";
  const std::string INSTRUCTION      = "Instruction";
  const std::string TASK_SUCCEEDED   = "Task_succeeded";
  const std::string TASK_FAILED      = "Task_failed";
  const std::string MISSION_COMPLETE = "Mission_complete";
  const std::string I_AM_READY     = "I_am_ready";
  const std::string ROOM_REACHED   = "Room_reached";
  const std::string OBJECT_GRASPED = "Object_grasped";
  const std::string TASK_FINISHED  = "Task_finished";
  const std::string ENVIRONMENT  = "Environment";
  const std::string GIVE_UP  = "Give_up";
  const std::string DOES_NOT_EXIST  = "Does_not_exist";
}

enum MAP {
  L20_01,
  L19_01,
  L21_01,
  L19_02,
};

enum PREPOSITION {
  DEFAULT_PREPOSITION,
  NEXT_TO_THE,
  ON_THE,
  IN_THE,
  UNDER_THE,
  CLOSE_TO_THE,
  BETWEEN,
  IN_FRONT_OF,
};

enum ROOM {
  DEFAULT_ROOM,
  KITCHEN,
  BEDROOM,
  LIVING_ROOM,
  LOBBY,
};

enum PLACE {
  DEFAULT_PLACE,
  SAFE_PLACE,
  CARDBOARD_BOX,
  TRASH_BOX_FOR_RECYCLE,
  TRASH_BOX_FOR_BURNABLE,
  TRASH_BOX_FOR_BOTTLE_CAN,
  WOODEN_SHELF,
  CORNER_SOFA,
  SQUARE_LOW_TABLE,
  ROUND_LOW_TABLE,
  WHITE_SIDE_TABLE,
  WOODEN_SIDE_TABLE,
  ARMCHAIR,
  WOODEN_BED,
  IRON_BED,
  DINING_TABLE,
  WAGON, // END OF DESTINATIONS
  DOOR,  // START OF OTHERS
  WHITE_CHAIR,
  WOODEN_CUPBOARD,
  BLUE_CUPBOARD,
  WHITE_RACK,
  CUSTOM_KITCHEN,
  REFRIGERATOR,
  OVEN,
  TOASTER,
  COFFEE_PRESS,
  FRYING_PAN,
  KETTLE,
  PAN,
  BIG_EMPTY_PLASTIC_BOTTLE,
  BIG_FILLED_PLASTIC_BOTTLE,
  WHITE_SHELF,
  WHITE_ROUND_TABLE,
  CHANGING_TABLE,
  WIDE_SHELF,
  SOFA,
  TV_RACK,
  CD_PLAYER,
  DESK_LAMP,
  FAN,
  FLOOR_LAMP,
  LAPTOP,
  SPEAKER,
  TV,
  TWIN_BELL_ALARM_CLOCK,
  BASKETBALL_BOARD,
  TOY_PLANE,
  CHESS_BOARD,
  DARTBOARD,
  BABY_BED,
  GRAND_PIANO,
  GUITAR,
  VIOLIN,
  BANANA,
  BELL_PEPPER,
  BRUSH,
  CUSHION,
  FRUIT_BASKET,
  VASE,
  CAMERA,
  ORANGE,
  WHITE_POTTED_PLANT,
  BROWN_POTTED_PLANT,
  WHEEL_CHAIR,
};

enum OBJECT {
  DEFAULT_OBJECT,
  NO_VALUE,
  WHITE_CUP,
  PINK_CUP,
  TUMBLER,
  EMPTY_KETCHUP,
  FILLED_KETCHUP,
  GROUND_PEPPER,
  SALT,
  SAUCE,
  SOYSAUCE,
  SUGAR,
  CANNED_JUICE,
  EMPTY_PLASTIC_BOTTLE,
  FILLED_PLASTIC_BOTTLE,
  CUBIC_CLOCK,
  BEAR_DOLL,
  DOG_DOLL,
  RABBIT_DOLL,
  TOY_CAR,
  TOY_PENGUIN,
  TOY_DUCK,
  NURSING_BOTTLE,
  APPLE,
  CIGARETTE,
  HOURGLASS,
  RUBIKS_CUBE,
  SPRAY_BOTTLE,
  GAME_CONTROLLER,
  PIGGY_BANK,
  MATRYOSHKA,
};

map<string, MAP> maps ({
  {"Layout2020HM01", MAP::L20_01},
  {"Layout2019HM01", MAP::L19_01},
  {"Layout2021HM01", MAP::L21_01},
  {"Layout2019HM02", MAP::L19_02},
});
map<MAP, string> mapsr ({
  {MAP::L20_01, "Layout2020HM01"},
  {MAP::L19_01, "Layout2019HM01"},
  {MAP::L21_01, "Layout2021HM01"},
  {MAP::L19_02, "Layout2019HM02"},
});

map<string, PREPOSITION> prepositions ({
  {"next_to_the", PREPOSITION::NEXT_TO_THE},
  {"on_the", PREPOSITION::ON_THE},
  {"in_the", PREPOSITION::IN_THE},
  {"under_the", PREPOSITION::UNDER_THE},
  {"close_to_the", PREPOSITION::CLOSE_TO_THE},
  {"between", PREPOSITION::BETWEEN},
  {"in_front_of", PREPOSITION::IN_FRONT_OF},
});
map<PREPOSITION, string> prepositionsr ({
  {PREPOSITION::NEXT_TO_THE, "next_to_the"},
  {PREPOSITION::ON_THE, "on_the"},
  {PREPOSITION::IN_THE, "in_the"},
  {PREPOSITION::UNDER_THE, "under_the"},
  {PREPOSITION::CLOSE_TO_THE, "close_to_the"},
  {PREPOSITION::BETWEEN, "between"},
  {PREPOSITION::IN_FRONT_OF, "in_front_of"},
});


map<string, ROOM> rooms ({
  {"kitchen", ROOM::KITCHEN},
  {"bedroom", ROOM::BEDROOM},
  {"living room", ROOM::LIVING_ROOM},
  {"lobby", ROOM::LOBBY},
});
map<ROOM, string> roomsr ({
  {ROOM::KITCHEN, "kitchen"},
  {ROOM::BEDROOM, "bedroom"},
  {ROOM::LIVING_ROOM, "living room"},
  {ROOM::LOBBY, "lobby"},
});

map<string, PLACE> places ({
  {"safe_place", PLACE::SAFE_PLACE},
  {"cardboard_box", PLACE::CARDBOARD_BOX},
  {"trash_box_for_recycle", PLACE::TRASH_BOX_FOR_RECYCLE},
  {"trash_box_for_burnable", PLACE::TRASH_BOX_FOR_BURNABLE},
  {"trash_box_for_bottle_can", PLACE::TRASH_BOX_FOR_BOTTLE_CAN},
  {"wooden_shelf", PLACE::WOODEN_SHELF},
  {"corner_sofa", PLACE::CORNER_SOFA},
  {"square_low_table", PLACE::SQUARE_LOW_TABLE},
  {"round_low_table", PLACE::ROUND_LOW_TABLE},
  {"white_side_table", PLACE::WHITE_SIDE_TABLE},
  {"wooden_side_table", PLACE::WOODEN_SIDE_TABLE},
  {"armchair", PLACE::ARMCHAIR},
  {"wooden_bed", PLACE::WOODEN_BED},
  {"iron_bed", PLACE::IRON_BED},
  {"dining_table", PLACE::DINING_TABLE},
  {"wagon", PLACE::WAGON},
  {"door", PLACE::DOOR},
  {"white_chair", PLACE::WHITE_CHAIR},
  {"wooden_cupboard", PLACE::WOODEN_CUPBOARD},
  {"blue_cupboard", PLACE::BLUE_CUPBOARD},
  {"white_rack", PLACE::WHITE_RACK},
  {"custom_kitchen", PLACE::CUSTOM_KITCHEN},
  {"refrigerator", PLACE::REFRIGERATOR},
  {"oven", PLACE::OVEN},
  {"toaster", PLACE::TOASTER},
  {"coffee_press", PLACE::COFFEE_PRESS},
  {"frying_pan", PLACE::FRYING_PAN},
  {"kettle", PLACE::KETTLE},
  {"pan", PLACE::PAN},
  {"big_empty_plastic_bottle", PLACE::BIG_EMPTY_PLASTIC_BOTTLE},
  {"big_filled_plastic_bottle", PLACE::BIG_FILLED_PLASTIC_BOTTLE},
  {"white_shelf", PLACE::WHITE_SHELF},
  {"white_round_table", PLACE::WHITE_ROUND_TABLE},
  {"changing_table", PLACE::CHANGING_TABLE},
  {"wide_shelf", PLACE::WIDE_SHELF},
  {"sofa", PLACE::SOFA},
  {"TV_rack", PLACE::TV_RACK},
  {"CD_player", PLACE::CD_PLAYER},
  {"desk_lamp", PLACE::DESK_LAMP},
  {"fan", PLACE::FAN},
  {"floor_lamp", PLACE::FLOOR_LAMP},
  {"laptop", PLACE::LAPTOP},
  {"speaker", PLACE::SPEAKER},
  {"TV", PLACE::TV},
  {"twin_bell_alarm_clock", PLACE::TWIN_BELL_ALARM_CLOCK},
  {"basketball_board", PLACE::BASKETBALL_BOARD},
  {"toy_plane", PLACE::TOY_PLANE},
  {"chess_board", PLACE::CHESS_BOARD},
  {"dartboard", PLACE::DARTBOARD},
  {"baby_bed", PLACE::BABY_BED},
  {"grand_piano", PLACE::GRAND_PIANO},
  {"guitar", PLACE::GUITAR},
  {"violin", PLACE::VIOLIN},
  {"banana", PLACE::BANANA},
  {"bell_pepper", PLACE::BELL_PEPPER},
  {"brush", PLACE::BRUSH},
  {"cushion", PLACE::CUSHION},
  {"fruit_basket", PLACE::FRUIT_BASKET},
  {"vase", PLACE::VASE},
  {"camera", PLACE::CAMERA},
  {"orange", PLACE::ORANGE},
  {"white_potted_plant", PLACE::WHITE_POTTED_PLANT},
  {"brown_potted_plant", PLACE::BROWN_POTTED_PLANT},
  {"wheel_chair", PLACE::WHEEL_CHAIR},
});
map<PLACE, string> placesr ({
  {PLACE::SAFE_PLACE, "safe_place"},
  {PLACE::CARDBOARD_BOX, "cardboard_box"},
  {PLACE::TRASH_BOX_FOR_RECYCLE, "trash_box_for_recycle"},
  {PLACE::TRASH_BOX_FOR_BURNABLE, "trash_box_for_burnable"},
  {PLACE::TRASH_BOX_FOR_BOTTLE_CAN, "trash_box_for_bottle_can"},
  {PLACE::WOODEN_SHELF, "wooden_shelf"},
  {PLACE::CORNER_SOFA, "corner_sofa"},
  {PLACE::SQUARE_LOW_TABLE, "square_low_table"},
  {PLACE::ROUND_LOW_TABLE, "round_low_table"},
  {PLACE::WHITE_SIDE_TABLE, "white_side_table"},
  {PLACE::WOODEN_SIDE_TABLE, "wooden_side_table"},
  {PLACE::ARMCHAIR, "armchair"},
  {PLACE::WOODEN_BED, "wooden_bed"},
  {PLACE::IRON_BED, "iron_bed"},
  {PLACE::DINING_TABLE, "dining_table"},
  {PLACE::WAGON, "wagon"},
  {PLACE::DOOR, "door"},
  {PLACE::WHITE_CHAIR, "white_chair"},
  {PLACE::WOODEN_CUPBOARD, "wooden_cupboard"},
  {PLACE::BLUE_CUPBOARD, "blue_cupboard"},
  {PLACE::WHITE_RACK, "white_rack"},
  {PLACE::CUSTOM_KITCHEN, "custom_kitchen"},
  {PLACE::REFRIGERATOR, "refrigerator"},
  {PLACE::OVEN, "oven"},
  {PLACE::TOASTER, "toaster"},
  {PLACE::COFFEE_PRESS, "coffee_press"},
  {PLACE::FRYING_PAN, "frying_pan"},
  {PLACE::KETTLE, "kettle"},
  {PLACE::PAN, "pan"},
  {PLACE::BIG_EMPTY_PLASTIC_BOTTLE, "big_empty_plastic_bottle"},
  {PLACE::BIG_FILLED_PLASTIC_BOTTLE, "big_filled_plastic_bottle"},
  {PLACE::WHITE_SHELF, "white_shelf"},
  {PLACE::WHITE_ROUND_TABLE, "white_round_table"},
  {PLACE::CHANGING_TABLE, "changing_table"},
  {PLACE::WIDE_SHELF, "wide_shelf"},
  {PLACE::SOFA, "sofa"},
  {PLACE::TV_RACK, "TV_rack"},
  {PLACE::CD_PLAYER, "CD_player"},
  {PLACE::DESK_LAMP, "desk_lamp"},
  {PLACE::FAN, "fan"},
  {PLACE::FLOOR_LAMP, "floor_lamp"},
  {PLACE::LAPTOP, "laptop"},
  {PLACE::SPEAKER, "speaker"},
  {PLACE::TV, "TV"},
  {PLACE::TWIN_BELL_ALARM_CLOCK, "twin_bell_alarm_clock"},
  {PLACE::BASKETBALL_BOARD, "basketball_board"},
  {PLACE::TOY_PLANE, "toy_plane"},
  {PLACE::CHESS_BOARD, "chess_board"},
  {PLACE::DARTBOARD, "dartboard"},
  {PLACE::BABY_BED, "baby_bed"},
  {PLACE::GRAND_PIANO, "grand_piano"},
  {PLACE::GUITAR, "guitar"},
  {PLACE::VIOLIN, "violin"},
  {PLACE::BANANA, "banana"},
  {PLACE::BELL_PEPPER, "bell_pepper"},
  {PLACE::BRUSH, "brush"},
  {PLACE::CUSHION, "cushion"},
  {PLACE::FRUIT_BASKET, "fruit_basket"},
  {PLACE::VASE, "vase"},
  {PLACE::CAMERA, "camera"},
  {PLACE::ORANGE, "orange"},
  {PLACE::WHITE_POTTED_PLANT, "white_potted_plant"},
  {PLACE::BROWN_POTTED_PLANT, "brown_potted_plant"},
  {PLACE::WHEEL_CHAIR, "wheel_chair"},
});

map<string, OBJECT> objects ({
  {"white_cup", OBJECT::WHITE_CUP}, 
  {"pink_cup", OBJECT::PINK_CUP}, 
  {"tumbler", OBJECT::TUMBLER}, 
  {"empty_ketchup", OBJECT::EMPTY_KETCHUP}, 
  {"filled_ketchup", OBJECT::FILLED_KETCHUP}, 
  {"ground_pepper", OBJECT::GROUND_PEPPER}, 
  {"salt", OBJECT::SALT}, 
  {"sauce", OBJECT::SAUCE}, 
  {"soysauce", OBJECT::SOYSAUCE}, 
  {"sugar", OBJECT::SUGAR}, 
  {"canned_juice", OBJECT::CANNED_JUICE}, 
  {"empty_plastic_bottle", OBJECT::EMPTY_PLASTIC_BOTTLE}, 
  {"filled_plastic_bottle", OBJECT::FILLED_PLASTIC_BOTTLE}, 
  {"cubic_clock", OBJECT::CUBIC_CLOCK}, 
  {"bear_doll", OBJECT::BEAR_DOLL}, 
  {"dog_doll", OBJECT::DOG_DOLL}, 
  {"rabbit_doll", OBJECT::RABBIT_DOLL}, 
  {"toy_car", OBJECT::TOY_CAR}, 
  {"toy_penguin", OBJECT::TOY_PENGUIN}, 
  {"toy_duck", OBJECT::TOY_DUCK}, 
  {"nursing_bottle", OBJECT::NURSING_BOTTLE}, 
  {"apple", OBJECT::APPLE}, 
  {"cigarette", OBJECT::CIGARETTE}, 
  {"hourglass", OBJECT::HOURGLASS}, 
  {"rubik's_cube", OBJECT::RUBIKS_CUBE}, 
  {"spray_bottle", OBJECT::SPRAY_BOTTLE},
  {"game_controller", OBJECT::GAME_CONTROLLER}, 
  {"piggy_bank", OBJECT::PIGGY_BANK}, 
  {"matryoshka", OBJECT::MATRYOSHKA}, 
});
map<OBJECT, string> objectsr ({
  {OBJECT::WHITE_CUP, "white_cup"}, 
  {OBJECT::PINK_CUP, "pink_cup"}, 
  {OBJECT::TUMBLER, "tumbler"}, 
  {OBJECT::EMPTY_KETCHUP, "empty_ketchup"}, 
  {OBJECT::FILLED_KETCHUP, "filled_ketchup"}, 
  {OBJECT::GROUND_PEPPER, "ground_pepper"}, 
  {OBJECT::SALT, "salt"}, 
  {OBJECT::SAUCE, "sauce"}, 
  {OBJECT::SOYSAUCE, "soysauce"}, 
  {OBJECT::SUGAR, "sugar"}, 
  {OBJECT::CANNED_JUICE, "canned_juice"}, 
  {OBJECT::EMPTY_PLASTIC_BOTTLE, "empty_plastic_bottle"}, 
  {OBJECT::FILLED_PLASTIC_BOTTLE, "filled_plastic_bottle"}, 
  {OBJECT::CUBIC_CLOCK, "cubic_clock"}, 
  {OBJECT::BEAR_DOLL, "bear_doll"}, 
  {OBJECT::DOG_DOLL, "dog_doll"}, 
  {OBJECT::RABBIT_DOLL, "rabbit_doll"}, 
  {OBJECT::TOY_CAR, "toy_car"}, 
  {OBJECT::TOY_PENGUIN, "toy_penguin"}, 
  {OBJECT::TOY_DUCK, "toy_duck"}, 
  {OBJECT::NURSING_BOTTLE, "nursing_bottle"}, 
  {OBJECT::APPLE, "apple"}, 
  {OBJECT::CIGARETTE, "cigarette"}, 
  {OBJECT::HOURGLASS, "hourglass"}, 
  {OBJECT::RUBIKS_CUBE, "rubik's_cube"}, 
  {OBJECT::SPRAY_BOTTLE, "spray_bottle"},
  {OBJECT::GAME_CONTROLLER, "game_controller"}, 
  {OBJECT::PIGGY_BANK, "piggy_bank"}, 
  {OBJECT::MATRYOSHKA, "matryoshka"}, 
});

struct NavPose
{
  NavPose(){}
  NavPose(float px, float py, float pz, float ox, float oy, float oz, float ow){
    val.header.frame_id = "map";
    val.pose.position.x = px;
    val.pose.position.y = py;
    val.pose.position.z = pz;
    val.pose.orientation.x = ox;
    val.pose.orientation.y = oy;
    val.pose.orientation.z = oz;
    val.pose.orientation.w = ow;
  }
  NavPose(float px, float py, float pz){
    val.header.frame_id = "map";
    val.pose.position.x = px;
    val.pose.position.y = py;
    val.pose.position.z = pz;
    val.pose.orientation.x = 0.0;
    val.pose.orientation.y = 0.0;
    val.pose.orientation.z = 0.0;
    val.pose.orientation.w = 0.0;
  } 
  geometry_msgs::PoseStamped val;
};

struct ObjectPlaceInfo
{
  ObjectPlaceInfo(){}
  ObjectPlaceInfo(PREPOSITION prep_, string ref_1_, string ref_2_, NavPose val_):
    prep(prep_), val(val_), ref_1(ref_1_), ref_2(ref_2_)
  {
    
  }
  NavPose val;
  PREPOSITION prep;
  string ref_1;
  string ref_2;
};

map<MAP, map<ROOM, map<PLACE, NavPose>>> NavPosesDict ({
  {
    MAP::L20_01,
    {
      {
        ROOM::LIVING_ROOM, {
          {PLACE::SAFE_PLACE, NavPose(0.5866, 1.672, 0.0, 0.0, 0.0, 0.0, 1.0)},
          {PLACE::SOFA, NavPose(-0.2026112, 1.795484, 0.0, 0.0, 0.0, 0.99999, 0.00122352)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(-0.287374, 3.7186, 0.0, 0.0, 0.0, 0.99999, 0.001487)},
          {PLACE::WIDE_SHELF, NavPose(-0.7077, -1.11033, 0.0, 0.0, 0.0, 0.99999, 0.70604)},
          {PLACE::DINING_TABLE, NavPose(0.937265, 2.0945, 0.0, 0.0, 0.0, -0.0802, 0.99678)},//TV SIDE
          {PLACE::DINING_TABLE, NavPose(2.48, -0.22875, 0.0, 0.0, 0.0, 0.70554, 0.708667)},//Moderator side
          {PLACE::WOODEN_SIDE_TABLE, NavPose(1.841446, 2.98928, 0.0, 0.0, 0.0, -0.704636, 0.709568)},
          {PLACE::WOODEN_SHELF, NavPose(2.41756, 4.13381, 0.0, 0.0, 0.0, 0.722, 0.69186)},
        }
      },
      {
        ROOM::BEDROOM, {
          {PLACE::SAFE_PLACE, NavPose(0.4847, 6.786, 0.0, 0.0, 0.0, -0.70817, 0.94428)},
          {PLACE::ROUND_LOW_TABLE, NavPose(-0.113027, 7.792, 0.0, 0.0, 0.0, 0.68877, 0.724975)},
          {PLACE::CARDBOARD_BOX, NavPose(1.1117, 8.1934, 0.0, 0.0, 0.0, 0.660465, 0.750856)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(2.762175, 7.51023, 0.0, 0.0, 0.0, -0.65174, 0.758448)},
          {PLACE::IRON_BED, NavPose(2.885, 8.004, 0.0, 0.0, 0.0, 0.012144, 0.9999)},
        }
      },
      {
        ROOM::KITCHEN, {
          {PLACE::SAFE_PLACE, NavPose(7.634, 1.482, 0.0, 0.0, 0.0, -0.566, 0.82442)},
          {PLACE::WOODEN_CUPBOARD, NavPose(7.833177, -1.74045, 0.0, 0.0, 0.0, -0.0304, 0.9999)},
          {PLACE::BLUE_CUPBOARD, NavPose(8.0, -0.2335, 0.0, 0.0, 0.0, 0.0472, 0.999)},
          {PLACE::CUSTOM_KITCHEN, NavPose(7.3316, 1.289, 0.0, 0.0, 0.0, -0.99999, 0.00361)},
        }
      },
    },
  },
  {
    MAP::L19_01,
    {
      {
        ROOM::LIVING_ROOM, {
          // USING
          {PLACE::SAFE_PLACE, NavPose(1, 0.256114661693573, 0.0, 0.0, 0.0, 0.0, 1)},
          // USING
          {PLACE::WHITE_SIDE_TABLE, NavPose(1.93948233127594, 0.256114661693573, 0.0, 0.0, 0.0, -0.6848205381255134, 0.7287117609600399)},
          {PLACE::SQUARE_LOW_TABLE, NavPose(1.6020655632019043, 3.8968348503112793, 0.0, 0.0, 0.0, -0.9996586767371922, 0.02612527557837206)},
          {PLACE::TV_RACK, NavPose(0.6039611101150513, 5.154543399810791, 0.0, 0.0, 0.0, 0.7128737867773152, 0.7012923528213972)},
          {PLACE::SOFA, NavPose(-0.17298543453216553, 4.791591644287109, 0.0, 0.0, 0.0, 0.9999678275452648, 0.008021463357986088)},
        }
      },
      {
        ROOM::LOBBY, {
          {PLACE::WHITE_RACK, NavPose(-0.4478911757469177, -3.0020151138305664, 0.0, 0.0, 0.0, 0.9973480412355308, 0.0727796993924122)},
          {PLACE::SOFA, NavPose(-0.2216578722000122, -4.56047248840332, 0.0, 0.0, 0.0, 0.9988937866888545, 0.047023429419823744)},
          {PLACE::CORNER_SOFA, NavPose(-0.2583094835281372, -5.568724632263184, 0.0, 0.0, 0.0, -0.7675427763141243, 0.6409977273969123)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(0.8920080065727234, -5.71284818649292, 0.0, 0.0, 0.0, -0.753874594052649, 0.6570183379799636)},
          {PLACE::WHEEL_CHAIR, NavPose(1.3988338708877563, -3.015367269515991, 0.0, 0.0, 0.0, 0.5288518825800079, 0.8487141369693223)},
          {PLACE::ARMCHAIR, NavPose(2.157485008239746, -3.1748836040496826, 0.0, 0.0, 0.0, 0.6018731742888666, 0.7985916867031886)},
        }
      },
      {
        ROOM::BEDROOM, {
          {PLACE::WOODEN_BED, NavPose(6.84346342086792, -4.616459369659424, 0.0, 0.0, 0.0, 0.7180732628866324, 0.6959675201669581)},
          {PLACE::WAGON, NavPose(7.958005905151367, -3.0086867809295654, 0.0, 0.0, 0.0, 0.7207458276922953, 0.6931994315232437)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(9.192727088928223, -2.893117904663086, 0.0, 0.0, 0.0, 0.7210581591748229, 0.6928745420979298)},
          {PLACE::CARDBOARD_BOX, NavPose(8.946081161499023, -4.72611141204834, 0.0, 0.0, 0.0, 0.027933683258877146, 0.9996097785333998)},
          {PLACE::BASKETBALL_BOARD, NavPose(8.879722595214844, -5.284809112548828, 0.0, 0.0, 0.0, 0.019264666649894242, 0.999814419089297)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(8.913117408752441, -6.205936431884766, 0.0, 0.0, 0.0, 0.09591208369898536, 0.9953898091705173)},
        }
      },
      {
        ROOM::KITCHEN, {
          {PLACE::BLUE_CUPBOARD, NavPose(8.849146842956543, 3.4084386825561523, 0.0, 0.0, 0.0, 0.25859203203004855, 0.9659866256685805)},
          {PLACE::CUSTOM_KITCHEN, NavPose(8.273064613342285, 4.6657915115356445, 0.0, 0.0, 0.0, -0.00944165733666463, 0.9999554265599727)}, ////////
          {PLACE::CUSTOM_KITCHEN, NavPose(7.793449878692627, 4.92230749130249, 0.0, 0.0, 0.0, 0.7034533644153533, 0.7107414185853531)}, ////////
          {PLACE::DINING_TABLE, NavPose(8.601373672485352, 3.3828446865081787, 0.0, 0.0, 0.0, -0.9932273818598938, 0.11618678033124367)},
          {PLACE::DINING_TABLE, NavPose(4.916623115539551, 3.558028221130371, 0.0, 0.0, 0.0, 0.006966960133528522, 0.9999757304387431)}, ////////
          // USING
          {PLACE::WOODEN_SIDE_TABLE, NavPose(4.82, 2.3, 0.0, 0.0, 0.0, -0.83, 0.6)},
        }
      }
    },
  },
  {
    MAP::L21_01,
    {
      {
        ROOM::LIVING_ROOM, {
          {PLACE::TV_RACK, NavPose(3.482243537902832, -2.5356180667877197, 0.0, 0.0, 0.0, 0.005560915195685532, 0.9999845379915564)},
          {PLACE::WOODEN_SHELF, NavPose(3.867281675338745, -0.4261610507965088, 0.0, 0.0, 0.0, 0.01900266466479881, 0.9998194330656097)}, // REVISAR ESTO es shelf_b en unity
          {PLACE::WHITE_SHELF, NavPose(4.194906711578369, 1.221889853477478, 0.0, 0.0, 0.0, -0.008950619652273227, 0.9999599424016146)},
          {PLACE::DINING_TABLE, NavPose(1.268209457397461, -2.180996894836426, 0.0, 0.0, 0.0, 0.006834070551170065, 0.9999766474671804)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(2.0867316722869873, 0.41760021448135376, 0.0, 0.0, 0.0, 0.6977546639982187, 0.7163368124483991)},
          {PLACE::WAGON, NavPose(-1.278163194656372, -2.337834358215332, 0.0, 0.0, 0.0, 0.9999733552967882, 0.007299910717488669)},
          {PLACE::SAFE_PLACE, NavPose(0.32328665256500244, -0.8963017463684082, 0.0, 0.0, 0.0, -0.007890638968023481, 0.9999688684237507)}, // SP3 Estos son nomás para diferenciar
        }
      },
      {
        ROOM::BEDROOM, {
          {PLACE::WHITE_SIDE_TABLE, NavPose(1.4373973608016968, -7.6284027099609375, 0.0, 0.0, 0.0, 0.7091910354336753, 0.7050163652430427)},
          {PLACE::WAGON, NavPose(0.6248307228088379, -9.161921501159668, 0.0, 0.0, 0.0, 0.9988429331242115, 0.04809152677782399)},
          {PLACE::WOODEN_BED, NavPose(0.6300791501998901, -9.319271087646484, 0.0, 0.0, 0.0, -0.7149225413615987,  0.6992036612125778)},
          {PLACE::ARMCHAIR, NavPose(3.998140573501587, -9.764022827148438, 0.0, 0.0, 0.0, -0.704819047564419, 0.7093871370347683)},
          {PLACE::SAFE_PLACE, NavPose(2.2497024536132812, -8.507503509521484, 0.0, 0.0, 0.0, 0.7057348114259083, 0.7084760941214866)}, // SP4
        }
      },
      {
        ROOM::LOBBY, {
          {PLACE::DINING_TABLE, NavPose(-3.2788901329040527, -6.431180000305176, 0.0, 0.0, 0.0, -0.6968113102128608, 0.7172544861898293)},
          {PLACE::WOODEN_SHELF, NavPose(-4.143185615539551, -4.954776287078857, 0.0, 0.0, 0.0, 0.6996604856258687, 0.7144754753340199)},
          {PLACE::CARDBOARD_BOX, NavPose(-5.843554496765137, -6.973861217498779, 0.0, 0.0, 0.0, 0.999993820853363, 0.003515430996656767)},
          {PLACE::SOFA, NavPose(-5.634425163269043, -9.220937728881836, 0.0, 0.0, 0.0, 0.9999697366360957, -0.007779833670300087)},
          {PLACE::CORNER_SOFA, NavPose(-5.264636516571045, -9.917207717895508, 0.0, 0.0, 0.0, -0.7178343205834292, 0.6962139672489532)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(-3.6912808418273926,  -9.940662384033203, 0.0, 0.0, 0.0, -0.7000785182684996, 0.7140658710924238)},
          {PLACE::SAFE_PLACE, NavPose(-4.726175785064697, -8.084532737731934, 0.0, 0.0, 0.0,  -0.6961813519917607, 0.7178659520682982)}, // SP1
          {PLACE::SAFE_PLACE, NavPose(-3.243220329284668, -6.015560150146484, 0.0, 0.0, 0.0,  -0.7088108411358857, 0.7053986046826561)}, // SP2
        }
      }
    },
  },
  {
    MAP::L19_02,
    {
      {
        ROOM::LIVING_ROOM, {
        {PLACE::SAFE_PLACE, NavPose(4.162288188934326, 10.344466209411621, 0.0, 0.0, 0.0, 0.9934225681989102, -0.1145058993811305)},
        {PLACE::TV_RACK, NavPose(2.6088528633117676,11.669910430908203, 0.0, 0.0, 0.0, 0.9316578557183771, 0.3633368132714268)},
        {PLACE::TV_RACK, NavPose(0.9409695267677307,11.956880569458008, 0.0, 0.0, 0.0, 0.7491518829672205, 0.6623982610534753)},
        {PLACE::SQUARE_LOW_TABLE, NavPose(1.0231002569198608, 9.958452224731445, 0.0, 0.0, 0.0,0.7078382135515991, 0.710416769872711)},
        {PLACE::ROUND_LOW_TABLE, NavPose(1.5209612846374512, 10.306236267089844, 0.0, 0.0, 0.0, -0.6922488051576812, 0.7216589164957106)},
        {PLACE::SOFA, NavPose(0.2898530960083008, 9.280439376831055, 0.0, 0.0, 0.0, 0.997100401065248,-0.0760972417077096)},
        {PLACE::WHITE_SIDE_TABLE, NavPose( 1.239661693572998, 8.285531997680664, 0.0, 0.0, 0.0, -0.7316714280301503, 0.6816574810007741)},
        {PLACE::CARDBOARD_BOX, NavPose(2.4921858310699463,  8.43100357055664, 0.0, 0.0, 0.0, -0.6535706856910457,  0.7568654826356771)},
        {PLACE::CARDBOARD_BOX, NavPose(2.4921858310699463,  8.43100357055664, 0.0, 0.0, 0.0, 0.09930598072220725,  0.9950569441960598)},
        }
      },
      {
        ROOM::LOBBY, {
          {PLACE::SAFE_PLACE, NavPose(1.0287795066833496, 4.384197235107422, 0.0, 0.0, 0.0, 0.9201866691651581, 0.39147987673791346)},
          {PLACE::WOODEN_SIDE_TABLE, NavPose(0.8149422407150269, 4.179503440856934, 0.0, 0.0, 0.0, 0.719431452364903, 0.694563449476090)},
          {PLACE::WOODEN_SHELF, NavPose(2.0481925010681152, 4.70300197601318, 0.0, 0.0, 0.0, 0.7251820342063223,  0.6885571997041207)},
          {PLACE::ARMCHAIR, NavPose(0.138410240411758, 4.4407854080200195, 0.0, 0.0, 0.0, 0.9996158141023578, 0.0277168576227620)},
          {PLACE::WAGON, NavPose(-0.0087454169988632, 3.457742929458618, 0.0, 0.0, 0.0, 0.9995234699575977,0.03086799319559702)},
          {PLACE::DINING_TABLE, NavPose(2.595471143722534,  2.214523792266845, 0.0, 0.0, 0.0, 0.9999856571458648,-0.00535588485246507)},
          {PLACE::WAGON, NavPose(2.814718246459961, 0.9436233043670654, 0.0, 0.0, 0.0, -0.13146172516337404, 0.991321247031995)},
          {PLACE::WAGON, NavPose(2.8941755294799805, -0.0017883479595184326, 0.0, 0.0, 0.0, 0.017947993421153406, 0.9998389217929827)},
          {PLACE::WHITE_SHELF, NavPose(0.032552480697631836, -0.34817367792129517, 0.0, 0.0, 0.0, -0.6943494717478347, 0.719637972235695)},
          {PLACE::WHITE_SIDE_TABLE, NavPose(1.503774881362915, 0.06274127960205078, 0.0, 0.0, 0.0, -0.7060605716201305, 0.7081514451044032)},
        }
      },
      {
        ROOM::KITCHEN, {
          {PLACE::SAFE_PLACE, NavPose(6.500889778137207, 2.382322311401367, 0.0, 0.0, 0.0, 0.1682089448325534, 0.9857513636198121)},
          {PLACE::DINING_TABLE, NavPose(7.074090003967285, 1.0631414651870728, 0.0, 0.0, 0.0, -0.7055270592713137, 0.7086829817598079)},
          {PLACE::TRASH_BOX_FOR_RECYCLE, NavPose(6.535140514373779,  4.1726813316345215, 0.0, 0.0, 0.0,0.7083570707408159, 0.705854276980377)},
          {PLACE::TRASH_BOX_FOR_BOTTLE_CAN, NavPose(7.507297992706299, 4.056127071380615, 0.0, 0.0, 0.0, 0.6865024510727213, 0.7271274885954634)},
          {PLACE::WOODEN_SIDE_TABLE, NavPose(8.413273811340332, 3.472461700439453, 0.0, 0.0, 0.0, -0.04044079334298124, 0.9991819365029525)},
          {PLACE::CUSTOM_KITCHEN, NavPose(8.018427848815918, 1.2233350276947021, 0.0, 0.0, 0.0, -0.0020697884384367963, 0.9999978579856159)},
          {PLACE::BLUE_CUPBOARD, NavPose(8.46084976196289,-1.3802666664123535, 0.0, 0.0, 0.0, 0.026881939232089133, 0.9996386153721365)},
          {PLACE::WOODEN_CUPBOARD, NavPose(8.789493560791016,-3.297832727432251, 0.0, 0.0, 0.0, 0.030552070891672608,  0.9995331765200344)},
          }
      }
    }
  }
});


map<MAP, map<ROOM, map<PLACE, vector<ObjectPlaceInfo>>>> PlacePosesDict ({
  {
    MAP::L20_01,
    {
      {
        ROOM::BEDROOM, {
          {
            PLACE::ROUND_LOW_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.2365, 8.565, 0.47)),
              ObjectPlaceInfo(PREPOSITION::NEXT_TO_THE, "cardboard_box", "", NavPose(0.462, 8.8027, 0.47)),
              ObjectPlaceInfo(PREPOSITION::NEXT_TO_THE, "white_round_table", "", NavPose(-0.548, 8.77, 0.47)),
            },
          },
          {
            PLACE::IRON_BED,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(3.8719, 8.31447, 0.63)),
              ObjectPlaceInfo(PREPOSITION::CLOSE_TO_THE, "changing_table", "", NavPose(3.75, 7.883, 0.63)),
            },
          },
          {
            PLACE::CARDBOARD_BOX,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.081096, 9.100317, 0.37)),
              ObjectPlaceInfo(PREPOSITION::IN_THE, "", "", NavPose(1.081096, 9.100317, 0.37)),
            },
          },
          {
            PLACE::WHITE_ROUND_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-1.06, 8.725, 0.75)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.8, 6.839, 0.52)),
              ObjectPlaceInfo(PREPOSITION::IN_FRONT_OF, "desk_lamp", "", NavPose(3.3717, 6.8425, 0.52)),
            },
          },
          {
            PLACE::CHANGING_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.49435, 9.1226, 0.89)),
            },
          },
        },
      },
      {
        ROOM::LIVING_ROOM, {
          {
            PLACE::SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.98, 1.223, 0.48)),
            },
          },
          {
            PLACE::WIDE_SHELF,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.778, -1.70576, 0.62)),
              //ObjectPlaceInfo(PREPOSITION::NEXT_TO_THE, "", "", NavPose(-0.382, -1.70, 1.03)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.876, 3.6965, 0.5)),
            },
          },
          {
            PLACE::DINING_TABLE, // tv side
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.6886, 2.32177, 0.76)),
            },
          },
          {
            PLACE::DINING_TABLE, //moderator side
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.97, 0.551, 0.76)),
            },
          },
          {
            PLACE::WOODEN_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.489, -1.9453, 0.39)),
            },
          },
          {
            PLACE::WOODEN_SHELF,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.5, 5.37, 0.69)),
            },
          },
        },
      },
      {
        ROOM::KITCHEN, {
          {
            PLACE::CUSTOM_KITCHEN,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(6.82, 0.87287, 0.86)),
            },
          },
          {
            PLACE::BLUE_CUPBOARD,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(8.7377, -0.3055, 0.72)),
            },
          },
          {
            PLACE::WOODEN_CUPBOARD,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(8.81089, -1.548, 0.715)),
            },
          },
        },
      },
    },
  },
  {
    MAP::L19_01,
    {
      {
        ROOM::LIVING_ROOM, {
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.2456798553466797, -0.5553549528121948, 0.5)),
            },
          },
          {
            PLACE::SQUARE_LOW_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.0706145763397217, 3.7684335708618164, 0.45)),
            },
          },
          {
            PLACE::TV_RACK,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(0.7550870776176453, 6.016347885131836, 0.78)),
            },
          },
          {
            PLACE::SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.8608369827270508, 4.738407135009766, 0.47)),
            },
          },
        },
      },
      {
        ROOM::LOBBY, {
          {
            PLACE::WHITE_RACK,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-1.1308220624923706, -3.2177672386169434, 0.65)),
            },
          },
          {
            PLACE::SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-1.0445051193237305, -4.528985977172852, 0.47)),
            },
          },
          {
            PLACE::CORNER_SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.6737898588180542, -6.312641143798828, 0.48)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(0.5969144701957703, -6.5032157897949222, 0.5)),
            },
          },
          {
            PLACE::WHEEL_CHAIR,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.3951780796051025, -1.9714635610580444, 0.35)),
            },
          },
          {
            PLACE::ARMCHAIR,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.424182415008545, -2.2470920085906982, 0.51)), /////
            },
          },
        },
      },
      {
        ROOM::BEDROOM, {
          {
            PLACE::WOODEN_BED,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(6.812292098999023, -3.7061290740966797, 0.56)),
            },
          },
          {
            PLACE::WAGON,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(8.128647804260254, -2.378086805343628, 0.735)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.518295288085938, -2.199258804321289, 0.5)),
            },
          },
          {
            PLACE::CARDBOARD_BOX,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.792860984802246, -4.719234943389893, 0.36)),
              ObjectPlaceInfo(PREPOSITION::IN_THE, "", "", NavPose(9.792860984802246, -4.719234943389893, 0.36)),
            },
          },
          {
            PLACE::BASKETBALL_BOARD,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.608718872070312, -5.155330181121826, 0.01)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.412614822387695, -6.353552341461182, 0.5)),
            },
          },
        },
      },
      {
        ROOM::KITCHEN, {
          {
            PLACE::BLUE_CUPBOARD,
            {
              ObjectPlaceInfo(PREPOSITION::IN_THE, "", "", NavPose(9.76361083984375, 3.664841413497925, 0.72)),
            },
          },
          {
            PLACE::CUSTOM_KITCHEN,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(7.690327167510986, 5.552192211151123, 0.868)), ////////
            },
          },
          {
            PLACE::DINING_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(7.624382972717285, 3.1180496215820312, 0.758)),
            },
          },
          {
            PLACE::DINING_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(5.7285332679748535, 3.512711763381958, 0.758)), ////////
            },
          },
          {
            // USING
            PLACE::WOODEN_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.75, 1.47, 0.40)),
            },
          },
        },
      },
    },
  },

  {
    MAP::L21_01,
    {
      {
        ROOM::LIVING_ROOM, {
          {
            PLACE::TV_RACK,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.215521812438965, -2.7095694541931152, 0.7732077836990356)),
            },
          },
          {
            PLACE::WOODEN_SHELF,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.40011739730835, -0.5169335603713989, 1.030427098274231)),
            },
          },
          {
            PLACE::WHITE_SHELF,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.72208833694458, 1.1209731101989746, 1.0003609657287598)),
            },
          },
          {
            PLACE::DINING_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.8404173851013184, -2.2469568252563477, 0.7539012432098389)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.0427439212799072, 1.0493993759155273, 0.4984264373779297)),
            },
          },
          {
            PLACE::WAGON,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-1.9974603652954102, -2.230645179748535, 0.7299420833587646)),
            },
          }
        },
      },
      {
        ROOM::BEDROOM, {
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.8149440288543701, -6.842770576477051, 0.5028526782989502)),
            },
          },
          {
            PLACE::WAGON,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.06300985813140869, -8.970041275024414, 0.7295196056365967)),
            },
          },
          {
            PLACE::WOODEN_BED,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(0.4290456771850586, -10.078874588012695, 0.5584485530853271)),
            },
          },
          {
            PLACE::ARMCHAIR,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(3.9283857345581055, -10.873620986938477, 0.5037961006164551)),
            },
          },
        },
      },
      {
        ROOM::LOBBY, {
          {
            PLACE::DINING_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-3.1999435424804688, -7.2605390548706055, 0.7522521018981934)),
            },
          },
          {
            PLACE::WOODEN_SHELF,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-4.271357536315918, -4.137674331665039, 1.01604425907135)),
            },
          },
          {
            PLACE::CARDBOARD_BOX,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-6.7405171394348145, -7.018037796020508, 0.003239154815673828)),
            },
          },
          {
            PLACE::SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-6.203764915466309, -9.2236909866333, 0.4684404134750366)),
            },
          },
          {
            PLACE::CORNER_SOFA,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-5.278865337371826, -10.622594833374023, 0.47045135498046875)),
            },
          },
          {
            PLACE::WHITE_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-3.9263648986816406, -10.548824310302734, 0.4963653087615967)),
            },
          },
        },
      },
    },  
  },
  {
    MAP::L19_02,
    {
      {
        ROOM::LIVING_ROOM, {
        {
          PLACE::TV_RACK, 
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(0.7862460613250732, 12.457925796508789, 0.8126804828643799)),
          }
        },////////
        {
          PLACE::SQUARE_LOW_TABLE,  
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.1186184883117676, 11.237810134887695, 0.4490177631378174)),
          }
        },
        {
          PLACE::ROUND_LOW_TABLE,  
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.376150369644165, 9.163229942321777, 0.4553685188293457)),
          }
        },
        {
          PLACE::SOFA, 
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.72208833694458, 1.1209731101989746, 1.0003609657287598)),
          }
        },
        {
          PLACE::WHITE_SIDE_TABLE,  
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.14431631565094, 7.07045078277587, 0.502256631851196)),
          }
        },
        {
          PLACE::CARDBOARD_BOX,  
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.360870838165283,6.877640247344971, 0.2942047119140625)),
          }
        },
        {
          PLACE::CARDBOARD_BOX,  
          {
            ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(5.51359748840332,11.727906227111816, 0.4285943508148193)),
          }
        },
        }
      },

      {
        ROOM::LOBBY, {
          {
            PLACE::WOODEN_SIDE_TABLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(0.8732515573501587, 5.662199497222,  0.39150428771972656)),
            }
          },
          {
            PLACE::WOODEN_SHELF,  
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(2.051006317138672, 5.753909111022, 0.683762788772583)),
            }
          },
          {
            PLACE::ARMCHAIR, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(-0.971174776554107, 4.374964714050, 0.514355659484863)),
            }
          },
          {
            PLACE::WAGON, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(4.72208833694458, 1.1209731101989746, 1.0003609657287598)),
            }
          },
          {
            PLACE::DINING_TABLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.5548590421676636, 1.9252166748046875, 0.7544512748718262)),
            }
          },
          {
            PLACE::WAGON, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(3.592669010162353, 0.803871631622314, 0.7320901155471802)),
            }
          },
          {
            PLACE::WAGON, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(3.53913497924804,  0.2434487342834472, 0.7330951690673828)),
            }
          },
          {
            PLACE::WHITE_SHELF, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose( -0.1612470149993896,-1.05848443508148, 0.6989071369171143)),
            }
          },
          {
            PLACE::WHITE_SIDE_TABLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.4420981407165527, -1.348539829254150, 0.501941204071044)),
            }
          },
        }
      },
      {
        ROOM::KITCHEN, {
          {
            PLACE::DINING_TABLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(7.613319396972656,-0.433400750160217, 0.8976042270660)),
            }
          },
          {
            PLACE::TRASH_BOX_FOR_RECYCLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(6.754265308380127, 4.807693004608154, 0.493165850639343)),
            }
          },
          {
            PLACE::TRASH_BOX_FOR_BOTTLE_CAN, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose( 7.480559349060059, 4.902200698852539,  0.5196051597595215)),
            }
          },
          {
            PLACE::WOODEN_SIDE_TABLE, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.330330848693848, 3.451228380203247, 0.3885672092437744)),
            }
          },
          {
            PLACE::CUSTOM_KITCHEN, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.3450326919555668, 0.71959626674652, 0.8593001365661621)),
            }
          },
          {
            PLACE::BLUE_CUPBOARD, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.523090362548828, -1.3136205673217773, 0.7166948318481445)),
            }
          },
          {
            PLACE::WOODEN_CUPBOARD, 
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(9.59984302520752,-3.160830020904541, 0.7158498764038086)),
            }
          },
        }
      }
    }
  }
});

#endif /* LABELS_H_ */
