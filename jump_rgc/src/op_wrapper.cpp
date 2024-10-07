#include "jump_rgc/op_wrapper.h"

Op_Wrapper::Op_Wrapper(JumpRobot *Robot, Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_qr)
    : q(_q), qd(_qd), qr(_qr), _JumpRobot(Robot)
{
    this->optP1 = new OptProblem1(Robot);
    this->op[0] = this->optP1;

    this->optP2 = new OptProblem2(Robot);
    this->op[1] = this->optP2;

    this->optP3 = new OptProblem3(Robot);
    this->op[2] = this->optP3;

    this->optP4 = new OptProblem4(Robot);
    this->op[3] = this->optP4;

    this->optP5 = new OptProblem5(Robot);
    this->op[4] = this->optP5;

    this->optP6 = new OptProblem6(Robot);
    this->op[5] = this->optP6;

    this->qhl.resize(2, 1);
}

Op_Wrapper::~Op_Wrapper()
{
}

void Op_Wrapper::RGCConfig(double _ts, double _Kp, double _Kd)
{
    // TODO - create a function that reads some loader file

    Eigen::MatrixXd Q, R, ref, Ub, Lb;

    Q.resize(2, 2);
    Q << 0.15, 0, 0, 0.15;

    R.resize(2, 2);
    R << 4.0, 0, 0, 4.0;

    Ub.resize(5, 1);
    Ub << _JumpRobot->qU, 0, OsqpEigen::INFTY, -this->g * 2.5 * _JumpRobot->m_total;

    Lb.resize(5, 1);
    Lb << _JumpRobot->qL, -OsqpEigen::INFTY, 0, -this->g * 0.5 * _JumpRobot->m_total;

    this->op[0]->SetInternalVariables();
    this->op[0]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->op[0]->UpdateModelConstants();
    this->ConfPO(0);
    this->op[0]->SetWeightMatrices(Q, R);
    this->op[0]->UpdateReferences();
    this->op[0]->SetConsBounds(Lb, Ub);

    // PO 1
    this->ClearPO();

    this->op[1]->SetInternalVariables();
    this->op[1]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->op[1]->UpdateModelConstants();
    this->ConfPO(1);
    this->op[1]->SetWeightMatrices(Q, R);
    this->op[1]->UpdateReferences();
    this->op[1]->SetConsBounds(Lb, Ub);

    // PO 2

    Ub.resize(4, 1);
    Ub << _JumpRobot->qU, 150, 150;

    Lb.resize(4, 1);
    Lb << _JumpRobot->qL, -150, -150;

    this->ClearPO();

    this->op[2]->SetInternalVariables();
    this->op[2]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->op[2]->UpdateModelConstants();
    this->ConfPO(2);
    this->op[2]->SetWeightMatrices(Q, R);
    this->op[2]->UpdateReferences();
    this->op[2]->SetConsBounds(Lb, Ub);

    // PO 3

    this->ClearPO();

    this->op[3]->SetInternalVariables();
    this->op[3]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->op[3]->UpdateModelConstants();
    this->ConfPO(3);
    this->op[3]->SetWeightMatrices(Q, R);
    this->op[3]->UpdateReferences();
    this->op[3]->SetConsBounds(Lb, Ub);

    // PO 4

    this->ClearPO();

    Q.resize(3, 3);
    Q << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.015;

    Ub.resize(5, 1);
    Ub << _JumpRobot->qU, 0, OsqpEigen::INFTY, -this->g * 3 * _JumpRobot->m_total;

    Lb.resize(5, 1);
    Lb << _JumpRobot->qL, -OsqpEigen::INFTY, 0, -this->g * 0.3 * _JumpRobot->m_total;

    Eigen::VectorXd nr;
    nr.resize(1, 1);
    nr.setZero();

    this->op[4]->SetInternalVariables();
    this->op[4]->SetConstants(_ts, 8, 4, _Kp, _Kd);
    this->op[4]->UpdateModelConstants();
    this->ConfPO(4);
    this->op[4]->SetWeightMatrices(Q, R);
    this->op[4]->UpdateReferences(nr);
    this->op[4]->SetConsBounds(Lb, Ub);

    // PO 5
    this->ClearPO();

    Ub.resize(4, 1);
    Ub << _JumpRobot->qU, 75, 75;

    Lb.resize(4, 1);
    Lb << _JumpRobot->qL, -75, -75;

    this->op[5]->SetInternalVariables();
    this->op[5]->SetConstants(_ts, 8, 4, _Kp, _Kd);
    this->op[5]->UpdateModelConstants();
    this->ConfPO(5);
    this->op[5]->SetWeightMatrices(Q, R);
    this->op[5]->UpdateReferences(nr);
    this->op[5]->SetConsBounds(Lb, Ub);

    this->first_conf = 1;
}

int Op_Wrapper::ChooseRGCPO(int npo)
{
    // verify if the PO is the same that the used before
    if (npo != this->last_op)
    {
        if (this->solver.isInitialized())
        {
            this->ClearPO();
        }
        this->ConfPO(npo);
        this->last_op = npo;
    }

    if (this->solver.isInitialized())
    {
        if (npo == 0 || npo == 1 || npo == 4 || npo == 5)
        {
            // update the states vector |dr, q, r, g, qa|
            this->x << _JumpRobot->com_vel, *(q), _JumpRobot->com_pos, this->g, *(qr);
        }

        if (npo == 2 || npo == 3)
        {
            // update the states vector |dr, q, qa|
            this->x << *(qd), *(q), *(qr);
        }

        this->qhl = this->op[npo]->qhl;
        this->op[npo]->UpdateStates(this->x);

        this->op[npo]->UpdateOptimizationProblem(this->H, this->F, this->Ain, this->Lb, this->Ub);

        if (this->SolvePO())
        {
            *(qr) = *(qr) + this->delta_qref;
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
        return -1;
}

void Op_Wrapper::ClearPO()
{
    this->solver.data()->clearLinearConstraintsMatrix();
    this->solver.data()->clearHessianMatrix();
    this->solver.clearSolver();
}

void Op_Wrapper::ConfPO(int index)
{
    // first, resize the matrices

    this->x.resize(this->op[index]->nxa);
    this->x.setZero();

    this->H.resize(this->op[index]->nu * this->op[index]->M, this->op[index]->nu * this->op[index]->M);
    this->H.setZero();

    this->F.resize(1, this->op[index]->nu * this->op[index]->M);
    this->F.setZero();

    this->Ain.resize(this->op[index]->nc * this->op[index]->N, this->op[index]->nu * this->op[index]->M);
    this->Ain.setZero();

    this->Lb.resize(this->op[index]->nc * this->op[index]->N);
    this->Lb.setZero();

    this->Ub.resize(this->op[index]->nc * this->op[index]->N);
    this->Ub.setZero();

    // then, configure the solver

    this->solver.settings()->setVerbosity(0);

    this->solver.data()->setNumberOfVariables(this->op[index]->nu * this->op[index]->M);

    this->hessian_sparse = this->H.sparseView();
    this->solver.data()->clearHessianMatrix();
    this->solver.data()->setHessianMatrix(this->hessian_sparse);

    this->solver.data()->setGradient(F.transpose());

    this->solver.data()->setNumberOfConstraints(this->op[index]->nc * this->op[index]->N);
    this->linearMatrix = this->Ain.sparseView();
    this->solver.data()->setLinearConstraintsMatrix(this->linearMatrix);
    this->solver.data()->setLowerBound(this->Lb);
    this->solver.data()->setUpperBound(this->Ub);

    if (this->op[index]->nc != 0)
        this->constraints = 1;

    if (!this->first_conf)
    {
        if (!this->solver.initSolver())
            std::cout << "***************** PO " << index << " Inicialization Problem ***************** " << std::endl;
        else
            std::cout << "***************** PO " << index << " OK ***************** " << std::endl;
    }
    else
    {
        if (!this->solver.initSolver())
        {
            std::cout << index << std::endl;
            // exit(0);
        }
    }
}

bool Op_Wrapper::SolvePO()
{

    this->hessian_sparse = this->H.sparseView();

    this->solver.updateHessianMatrix(this->hessian_sparse);
    this->solver.updateGradient(this->F.transpose());

    if (this->constraints != 0)
    {
        this->linearMatrix = this->Ain.sparseView();
        this->solver.updateLinearConstraintsMatrix(this->linearMatrix);
        this->solver.updateBounds(this->Lb, this->Ub);
    }

    if (this->solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError)
    {
        if (this->solver.getStatus() != OsqpEigen::Status::Solved)
        {
            if (this->debug)
                std::cout << "Not solved" << std::endl;
            return 0;
        }

        this->QPSolution = this->solver.getSolution();
        this->delta_qref = this->QPSolution.block(0, 0, 2, 1);
        if (this->debug)
            std::cout << "Solved" << std::endl;
        return 1;
    }
    else
    {
        if (this->debug)
            std::cout << "Not solved" << std::endl;
        return 0;
    }
}