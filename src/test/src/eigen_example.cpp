 Eigen::Matrix<double, 2, 3> my_matrix;
  my_matrix << 1, -2, 3, 4, 5, 6;
  std::cout <<"/////transpose//////"<< std::endl;
  std::cout << my_matrix.transpose()<< std::endl;
  std::cout<<my_matrix.minCoeff(&i, &j)<<std::endl;
  std::cout<<my_matrix.maxCoeff(&i, &j)<<std::endl;
  std::cout<<my_matrix.prod()<<std::endl;
  std::cout<<my_matrix.sum()<<std::endl;
  std::cout << my_matrix.array() - 2 << std::endl; // resta a la matrix -2 a cada elemento 
  std::cout << my_matrix.array().abs() << std::endl;

  std::cout << my_matrix << std::endl;

  std::cout <<"///////////////////Initialization//////////////////"<< std::endl;
    Eigen::Matrix2d a_2d; //a_2d  is a 2x2 matrix double
    a_2d.setRandom();
    std::cout << a_2d << std::endl;
 
    a_2d.setConstant(4.3);
    std::cout << a_2d << std::endl;

 
    Eigen::MatrixXd Midentity=Eigen::MatrixXd::Identity(6,6);
  	std::cout << Midentity << std::endl;
    Eigen::MatrixXd Mzeros=Eigen::MatrixXd::Zero(3, 3);
    std::cout << Mzeros << std::endl;

    Eigen::ArrayXXf table(10, 4);
    table.col(0) = Eigen::ArrayXf::LinSpaced(10, 0, 90);

    std::cout <<"//////////////////Block Elements Access////////////////////"<< std::endl;
    Eigen::MatrixXf mat(4, 4);
    mat << 1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12,
        13, 14, 15, 16;
    std::cout << mat << std::endl;
    std::cout << "Block in the middle" << std::endl;
    std::cout << mat.block<2, 2>(1, 1) << std::endl;

    std::cout <<"/////build matrix from vector, resizing matrix, dynamic size//////"<< std::endl;

    Eigen::MatrixXd dynamicMatrix;
    int rows, cols;
    rows=3;
    cols=2;
    dynamicMatrix.resize(rows,cols);
    dynamicMatrix<<-1,7,3,4,5,1;
    std::cout << dynamicMatrix << std::endl;

    std::cout <<"/////operation//////"<< std::endl;
	Eigen::Matrix4f M1;
	M1.setRandom();
	Eigen::Matrix4f M2(4, 4);
	M2 << 1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12,
        13, 14, 15, 16;

    std::cout << M1 << std::endl;
    std::cout << M2 << std::endl;
	std::cout <<"/////addition//////"<< std::endl;
	std::cout << M1+M2 << std::endl;
	std::cout <<"/////product//////"<< std::endl;
    std::cout << M1*M2 << std::endl;
    std::cout <<"/////product and transpose//////"<< std::endl;
    std::cout << M1*M2.transpose() << std::endl;
    std::cout <<"/////product dot//////"<< std::endl;
    //std::cout << M1.dot(M2) << std::endl;