// Utility function for checking out the repo since we want all the branches and tags
def checkoutRepo() {
  // Clean the workspace
  cleanWs()

  // Handles both PRs and branches
  script {
    if (env.CHANGE_BRANCH) {
      BRANCH_NAME_REAL = env.CHANGE_BRANCH
    } else {
      BRANCH_NAME_REAL = env.BRANCH_NAME
    }
  }

  // Checkout the code from github
  checkout([  $class: 'GitSCM',
    branches: [
        [name: 'refs/heads/' + BRANCH_NAME_REAL]
    ],
    userRemoteConfigs: [[credentialsId: 'Github_User_And_Token', url: 'https://github.com/LORD-MicroStrain/libmip.git']],
    extensions: [
    ]
  ])
}

pipeline {
  agent none
  stages {
    stage('Build') {
      // Run the windows and linux builds in parallel
      parallel {
        stage('Windows x86') {
          agent { label 'windows10' }
          options { skipDefaultCheckout() }
          steps {
            checkoutRepo()
            powershell """
              mkdir build_Win32
              cd build_Win32
              cmake .. `
                -A "Win32" `
                -T "v142" `
                -DBUILD_DOCUMENTATION=ON `
                -DBUILD_PACKAGE=ON
              if(\$?) { cmake --build . }
              if(\$?) { cmake --build . --target package }
              if(\$?) { cmake --build . --target package_docs }
            """
            archiveArtifacts artifacts: 'build_Win32/mipsdk_*'
          }
        }
        stage('Windows x64') {
          agent { label 'windows10' }
          options { skipDefaultCheckout() }
          steps {
            checkoutRepo()
            powershell """
              mkdir build_x64
              cd build_x64
              cmake .. `
                -A "x64" `
                -T "v142" `
                -DBUILD_PACKAGE=ON
              if(\$?) { cmake --build . }
              if(\$?) { cmake --build . --target package }
            """
            archiveArtifacts artifacts: 'build_x64/mipsdk_*'
          }
        }
        stage('Ubuntu amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            checkoutRepo()
            sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64"
            archiveArtifacts artifacts: 'build_ubuntu_amd64/mipsdk_*'
          }
        }
        stage('Centos amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            checkoutRepo()
            sh "./.devcontainer/docker_build.sh --os centos --arch amd64"
            archiveArtifacts artifacts: 'build_centos_amd64/mipsdk_*'
          }
        }
        // stage('Ubuntu arm64') {
        //   agent { label 'linux-arm64' }
        //   options { skipDefaultCheckout() }
        //   steps {
        //     checkoutRepo()
        //     sh "./.devcontainer/docker_build.sh --os ubuntu --arch arm64v8"
        //     archiveArtifacts artifacts: 'build_ubuntu_arm64v8/mipsdk_*'
        //   }
        // }
        // stage('Ubuntu arm32') {
        //   agent { label 'linux-arm64' }
        //   options { skipDefaultCheckout() }
        //   steps {
        //     checkoutRepo()
        //     sh "./.devcontainer/docker_build.sh --os ubuntu --arch arm32v7"
        //     archiveArtifacts artifacts: 'build_ubuntu_arm32v7/mipsdk_*'
        //   }
        // }
      }
    }
  }
  post {
    success {
      script {
        if (BRANCH_NAME && BRANCH_NAME == 'develop') {
          node("linux-amd64") {
            dir("/tmp/mip_sdk_${env.BRANCH_NAME}_${currentBuild.number}") {
              copyArtifacts(projectName: "${env.JOB_NAME}", selector: specific("${currentBuild.number}"));
              withCredentials([string(credentialsId: 'Github_Token', variable: 'GH_TOKEN')]) {
                sh '''
                  # Release to github
                  "${WORKSPACE}/scripts/release.sh" \
                    --artifacts "$(find "$(pwd)" -type f)" \
                    --target "${BRANCH_NAME}" \
                    --release "latest" \
                    --docs-zip "$(find "$(pwd)" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)" \
                    --generate-notes
                '''
              }
            }
          }
        } else if (BRANCH_NAME && BRANCH_NAME == 'master') {
          node("linux-amd64") {
            dir("/tmp/mip_sdk_${env.BRANCH_NAME}_${currentBuild.number}") {
              copyArtifacts(projectName: "${env.JOB_NAME}", selector: specific("${currentBuild.number}"));
              withCredentials([string(credentialsId: 'Github_Token', variable: 'GH_TOKEN')]) {
                sh '''
                # Release to the latest version if the master commit matches up with the commit of that version
                if (cd "${WORKSPACE}" && git describe --exact-match --tags HEAD &> /dev/null); then
                  # Publish a release
                  ${WORKSPACE}/scripts/release.sh" \
                    --artifacts "$(find "$(pwd)" -type f)" \
                    --target "${BRANCH_NAME}" \
                    --release "$(cd ${WORKSPACE} && git describe --exact-match --tags HEAD)" \
                    --docs-zip "$(find "$(pwd)" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)"
                else
                  echo "Not releasing from ${BRANCH_NAME} since the current commit does not match the latest released version commit"
                fi
                '''
              }
            }
          }
        }
      }
    }
  }
}
