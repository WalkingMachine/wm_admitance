/**
 * \file    ConfigManager.h
 * \author  Jeremie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \coauthor Francis Masse <francis.masse05@gmail.com>
 * \date    10/17/16
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <yaml-cpp/yaml.h>

namespace wm_admittance
{
    namespace utilities
    {
        /**
         * \brief Classe qui permet de gérer un fichier de reconfiguration dynamique
         * \tparam DynamicReconfigureType Type de l'objet de reconfiguration dynamique
         *
         * \note Cette classe démarre un serveur de reconfiguration.
         */
        template <class DynamicReconfigureType>
        class ConfigManager 
        {
        protected:

            /**
             * \brief Consutrcteur par défaut
             * \param[in] pManagerName Le nom du gestionnaire de configuration
             */
            ConfigManager(const std::string& pManagerName) :
                aManageName(pManagerName),
                aServer(ros::NodeHandle(std::string("~/") + aManageName)),
                aCurrentConfig()
            {
                // No logic body 
            }

            void init();

            virtual void onDynamicReconfigureChange(const DynamicReconfigureType& pConfig) = 0;
            virtual void writeConfigFile(const DynamicReconfigureType& pConfig) = 0;
            virtual void readConfigFile(DynamicReconfigureType& pConfig) = 0;

            std::string getManagerName();

        private:
            void dynamicReconfigureCallback(DynamicReconfigureType& pConfig, uint32_t pLevel);

            std::string aManageName;
            dynamic_reconfigure::Server<DynamicReconfigureType> aServer;
            DynamicReconfigureType aCurrentConfig;
            bool aIsInitializing;
        };

        /**
         * \brief Retourne le nom du gestionnaire de configuration
         * \return Le nom du gestionnaire de configuration
         */
        template <class DynamicReconfigureType>
        inline std::string ConfigManager<DynamicReconfigureType>::getManagerName()
        {
            return aManageName;
        }

        /**
         * \brief Retourne le nom du gestionnaire de configuration
         * \param[in] pConfig L'objet de configuration
         * \param pLevel Le niveau de configuration
         */
        template <class DynamicReconfigureType>
        inline void ConfigManager<DynamicReconfigureType>::dynamicReconfigureCallback(DynamicReconfigureType& pConfig,
                                                                                      uint32_t pLevel)
        {
            // Prevent warning;
            (void) pLevel;
            if (!aIsInitializing)
            {
                // Save new config
                aCurrentConfig = pConfig;
                // Calls the manager so it update itself
                onDynamicReconfigureChange(aCurrentConfig);
                // Save the configuration to file.
                writeConfigFile(aCurrentConfig);
            }
            else
            {
                pConfig = aCurrentConfig;
            }
        }

        /**
         * \brief Initialise le server de reconfiguration
         */
        template <class DynamicReconfigureType>
        inline void ConfigManager<DynamicReconfigureType>::init()
        {
            aIsInitializing = true;
            // Load the configuration from file.
            readConfigFile(aCurrentConfig);

            // The original configuration as default.
            aServer.setConfigDefault(aCurrentConfig);

            // Set the callback.
            aServer.setCallback(boost::bind(&ConfigManager<DynamicReconfigureType>::dynamicReconfigureCallback, this, _1, _2));

            aIsInitializing = false;

            // This makes sure the node has the good configuration.
            dynamicReconfigureCallback(aCurrentConfig, 0);
        }
    } // namespace utilities
} // namespace wm_admittance

#endif //CONFIG_MANAGER_H
