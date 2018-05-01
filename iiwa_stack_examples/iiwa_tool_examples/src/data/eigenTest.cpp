#include <iostream>
#include <iomanip>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>


int readFile_nlines(const std::string);
void readFile(Eigen::MatrixXf&, const std::string);

int main()
{
  int n_data = 0;

  n_data = readFile_nlines("desired_velocity.txt");
  std::cout << "N = " << n_data << std::endl; 
  Eigen::MatrixXf data(7, n_data);

  readFile(data, "desired_velocity.txt");
  std::cout << data << std::endl;

}

int readFile_nlines(const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }
    
    while (std::getline(inFile, str)) {
      n_data += 1;
    }
    
    inFile.close();
    
    return n_data;
}


void readFile(Eigen::MatrixXf &mat, const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;
    

    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    boost::char_separator<char> sep(" "); //Use space as the delimiter
    while (std::getline(inFile, str)) {
      std::vector<float> line;
      tokenizer tok(str, sep);
      // 
      std::transform( tok.begin(), tok.end(), std::back_inserter(line), 
                    &boost::lexical_cast<float,std::string> );
      int t = 0;
      for(std::vector<float>::iterator it = line.begin(); it != line.end(); ++it) {
        mat(t,n_data) = *it;
        // std::cout << line.size() << std::endl;
        t += 1;
      }
 
      n_data += 1;
      // std::copy(line.begin(), line.end(), std::ostream_iterator<float>(std::cout,"\n") ); //Print those for testing
    }

    inFile.close();
    // std::cout << "Sum = " << n_data << std::endl; 
    
}
