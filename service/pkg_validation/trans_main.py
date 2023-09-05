import numpy as np
from Full2Scale_Trans_def import full2scale_CCS
from Full2Scale_Trans_def import full2scale_SIMMAN

Data_List_6700t = ["CCS_bulk_6700t_turning_port_15", "CCS_bulk_6700t_turning_port_25",
                   "CCS_bulk_6700t_turning_port_35", "CCS_bulk_6700t_turning_star_15",
                   "CCS_bulk_6700t_turning_star_25", "CCS_bulk_6700t_turning_star_35",
                   "CCS_bulk_6700t_zigzag_port_10", "CCS_bulk_6700t_zigzag_port_15",
                   "CCS_bulk_6700t_zigzag_port_20", "CCS_bulk_6700t_zigzag_star_10",
                   "CCS_bulk_6700t_zigzag_star_15", "CCS_bulk_6700t_zigzag_star_20"]

Data_List_3500t = ["CCS_tanker_3500t_turning_port_15", "CCS_tanker_3500t_turning_port_25",
                   "CCS_tanker_3500t_turning_port_35", "CCS_tanker_3500t_turning_star_15",
                   "CCS_tanker_3500t_turning_star_25", "CCS_tanker_3500t_turning_star_35",
                   "CCS_tanker_3500t_zigzag_port_10", "CCS_tanker_3500t_zigzag_port_15",
                   "CCS_tanker_3500t_zigzag_port_20", "CCS_tanker_3500t_zigzag_star_10",
                   "CCS_tanker_3500t_zigzag_star_15", "CCS_tanker_3500t_zigzag_star_20"]

Data_List_KVLCC2 = ["SIMMAN2008_tanker_KVLCC2_MARIN_turning_port_35",
                    "SIMMAN2008_tanker_KVLCC2_MARIN_turning_star_35",
                    "SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_port_10",
                    "SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_port_20",
                    "SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_star_10",
                    "SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_star_20",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_turning_port_35",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_turning_star_35",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_port_10",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_port_20",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_turning_port_35",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_turning_star_35",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_port_10",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_port_20",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_star_10",
                    "SIMMAN2020_tanker_KVLCC2_MARIN_zigzag_star_20"]

Data_List_KCS = ["SIMMAN2020_container_KCS_MARIN_turning_port_35",
                 "SIMMAN2020_container_KCS_MARIN_turning_star_35",
                 "SIMMAN2020_container_KCS_MARIN_zigzag_port_10",
                 "SIMMAN2020_container_KCS_MARIN_zigzag_port_20",
                 "SIMMAN2020_container_KCS_MARIN_zigzag_star_10",
                 "SIMMAN2020_container_KCS_MARIN_zigzag_star_20"]

scale_factor_3500t = 22.815
scale_factor_6700t = 24.272
scale_factor_KVLCC2 = 45.714
scale_factor_KCS = 75.50

full2scale_CCS(Data_List_3500t, scale_factor_3500t)

full2scale_CCS(Data_List_6700t, scale_factor_6700t)

full2scale_SIMMAN(Data_List_KVLCC2, scale_factor_KVLCC2)

full2scale_SIMMAN(Data_List_KCS, scale_factor_KCS)