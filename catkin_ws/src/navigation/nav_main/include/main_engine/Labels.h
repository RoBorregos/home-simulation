#ifndef NAV_POSES_H_
#define NAV_POSES_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>

using namespace std;


enum MAP {
  L20_01,
  L19_01,
  L21_01,
  L19_02,
};

enum PREPOSITION {
  NEXT_TO_THE,
  ON_THE,
  IN_THE,
  UNDER_THE,
  CLOSE_TO_THE,
  BETWEEN,
  IN_FRONT_OF,
};

enum ROOM {
  KITCHEN,
  BEDROOM,
  LIVING_ROOM,
  LOBBY,
};

enum PLACE {
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
  {"Layout2019HM02", MAP::L19_01},
});

map<string, ROOM> rooms ({
  {"kitchen", ROOM::KITCHEN},
  {"bedroom", ROOM::BEDROOM},
  {"living room", ROOM::LIVING_ROOM},
  {"lobby", ROOM::LOBBY},
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

// Posiciones en todos los PLACES  de cada cuarto con una distancia de 50cm aproximadamente;
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
  }
});


// Definir para cada DESTINATION (13)
// el mayor número de preposiciones posibles considerando todos los PLACES.
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
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(1.081096, 9.100317, 0.4)),
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
  }
});

#endif /* NAV_POSES_H_ */
