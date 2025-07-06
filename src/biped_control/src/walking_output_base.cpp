#include "biped_control/walking_output_base.hpp"

void WalkingOutputBase::ConfigBase::InitConfigBase(const std::string &config_file, const RobotType &robot_type)
{
   yaml_parser.Init(config_file);

   dt_lowpass = yaml_parser.get_double("pose_command/dt_lowpass");
   velX_dt_cutoff = yaml_parser.get_double("pose_command/velX_dt_cutoff");
   velY_dt_cutoff = yaml_parser.get_double("pose_command/velY_dt_cutoff");
   vx_offset = yaml_parser.get_double("pose_command/vx_offset");
   vy_offset = yaml_parser.get_double("pose_command/vy_offset");
   TSS = yaml_parser.get_double("pose_command/TSS");
   TDS = yaml_parser.get_double("pose_command/TDS");
   znom = yaml_parser.get_double("pose_command/znom");
   zsw_max = yaml_parser.get_double("pose_command/zsw_max");
   zsw_neg = yaml_parser.get_double("pose_command/zsw_neg");
   stepWidthNominal = yaml_parser.get_double("pose_command/stepWidthNominal");
   bezierSwingHorizontal = yaml_parser.get_VectorXd("pose_command/bezierSwingHorizontal");
   bezierComVertical = yaml_parser.get_VectorXd("pose_command/bezierComVertical");
   maxStepSize = yaml_parser.get_double("pose_command/maxStepSize");
   velXmax = yaml_parser.get_double("pose_command/velXmax");
   velYmax = yaml_parser.get_double("pose_command/velYmax");

   fric_params.frictionCoef = yaml_parser.get_double("friction/frictionCoef");
   fric_params.Lfront = yaml_parser.get_double("friction/Lfront");
   fric_params.Lback = yaml_parser.get_double("friction/Lback");
   if (robot_type == RobotType::PlaneFoot)
   {
      fric_params.W = yaml_parser.get_double("friction/W");
   }
   fric_params.Fz_lb = yaml_parser.get_double("friction/Fz_lb");
   fric_params.Rot_frictionCoef = yaml_parser.get_double("friction/Rot_frictionCoef");
}

void WalkingOutputBase::ComputeHolonomicConstraints()
{
   MatrixXd Jh_internal, Jh_contact;
   VectorXd dJhdq_internal, dJhdq_contact;
   robot_->GetInternalHolonomicConstraints(Jh_internal, dJhdq_internal);

   robot_->GetContactHolonomicConstraints(domain.leftC, domain.rightC, Jh_contact, dJhdq_contact);
   Jh.resize(Jh_internal.rows() + Jh_contact.rows(), robot_->nv());
   Jh << Jh_internal, Jh_contact;
   dJhdq.resize(dJhdq_internal.size() + dJhdq_contact.size());
   dJhdq << dJhdq_internal, dJhdq_contact;
}

void WalkingOutputBase::ComputeFrictionConstriants(const FrictionParams& fric_params)
{
   robot_->GetFrictionCone(fric_params,domain.leftC, domain.rightC, Afric, bfric_ub);
}