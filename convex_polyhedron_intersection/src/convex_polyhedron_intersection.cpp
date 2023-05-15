#include <convex_polyhedron_intersection/convex_polyhedron_intersection.h>

namespace convex_polyhedron_intersection {

  bool intersection(const Eigen::MatrixXd& In1, const Eigen::MatrixXd& In2, Eigen::MatrixXd& Out, bool verbose){
    const int dim = In1.rows();

    const Eigen::MatrixXd R_nonneg(dim,0);
    const Eigen::MatrixXd R_free(dim,0);

    Eigen::MatrixXd A_eq1;
    Eigen::VectorXd b_eq1;
    Eigen::MatrixXd A_ineq1;
    Eigen::VectorXd b_ineq1;
    if(!cddeigen::VtoHgmp(In1,
                       R_nonneg,
                       R_free,
                       A_eq1,
                       b_eq1,
                       A_ineq1,
                       b_ineq1,
                       verbose)) return false;

    Eigen::MatrixXd A_eq2;
    Eigen::VectorXd b_eq2;
    Eigen::MatrixXd A_ineq2;
    Eigen::VectorXd b_ineq2;
    if(!cddeigen::VtoHgmp(In2,
                       R_nonneg,
                       R_free,
                       A_eq2,
                       b_eq2,
                       A_ineq2,
                       b_ineq2,
                       verbose)) return false;

    Eigen::MatrixXd A_eq(A_eq1.rows() + A_eq2.rows(), dim);
    A_eq.topRows(A_eq1.rows()) = A_eq1;
    A_eq.bottomRows(A_eq2.rows()) = A_eq2;
    Eigen::VectorXd b_eq(b_eq1.rows() + b_eq2.rows());
    b_eq.head(b_eq1.rows()) = b_eq1;
    b_eq.tail(b_eq2.rows()) = b_eq2;
    Eigen::MatrixXd A_ineq(A_ineq1.rows() + A_ineq2.rows(), dim);
    A_ineq.topRows(A_ineq1.rows()) = A_ineq1;
    A_ineq.bottomRows(A_ineq2.rows()) = A_ineq2;
    Eigen::VectorXd b_ineq(b_ineq1.rows() + b_ineq2.rows());
    b_ineq.head(b_ineq1.rows()) = b_ineq1;
    b_ineq.tail(b_ineq2.rows()) = b_ineq2;

    Eigen::MatrixXd R_nonneg3;
    Eigen::MatrixXd R_free3;
    if(!cddeigen::HtoVgmp(A_eq,
                       b_eq,
                       A_ineq,
                       b_ineq,
                       Out,
                       R_nonneg3,
                       R_free3,
                       verbose)) return false;

    return true;
  }

};
