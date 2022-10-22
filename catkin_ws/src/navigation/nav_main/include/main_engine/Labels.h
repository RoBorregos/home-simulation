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
  },
  {
    MAP::L19_01,
    {
      {
        ROOM::LIVING_ROOM, {
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
          {PLACE::WOODEN_SIDE_TABLE, NavPose(4.918543338775635, 2.2396326065063477, 0.0, 0.0, 0.0, -0.7087806941997595, 0.705428896154465)},
        }
      }
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
            PLACE::WOODEN_SIDE_TABLE,
            {
              ObjectPlaceInfo(PREPOSITION::ON_THE, "", "", NavPose(5.072391033172607, 1.542228102684021, 0.40)),
            },
          },
        },
      },
    },
  }
});

#endif /* NAV_POSES_H_ */
