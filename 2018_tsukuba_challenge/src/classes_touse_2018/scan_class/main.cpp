////////////////////////////////////////////////////////////////////////////////
//
// 作成開始日 2018 09 12 (水)
// ver.1完成日
//
//
//
// 作成者 : 福田　優人
//
//
////////////////////////////////////////////////////////////////////////////////

#include <Mlib/libfm.hpp>
#include <2018_tsukuba_challenge/scan_class.hpp>


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "scan_class");

	std::string topic = "/scan";

	scan_class SC_(topic, 1);

	while(ros::ok()){
    {
      int x;
      std::cout << "整数値を入力" << std::endl;
      std::cin >> x;
      switch (x) {
        case 0:{
					SC_.scan_Situation();
          break;
        }

        case 1:{

          break;
        }

        case -1:{

          break;
        }
      }
    }
  }
	return 0;


	return 0;
}
