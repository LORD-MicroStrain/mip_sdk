// Utility function for getting the real branch name even in a pull request
def branchName() {
  if (env.CHANGE_BRANCH) {
    return env.CHANGE_BRANCH
  } else {
    return env.BRANCH_NAME
  }
}

// Utility function for checking out the repo since we want all the branches and tags
def checkoutRepo() {
  // Clean the workspace
  cleanWs()

  // Handles both PRs and branches
  script {
    BRANCH_NAME_REAL = branchName();
  }

  // Checkout the code from github
  checkout([  $class: 'GitSCM',
    branches: [
        [name: 'refs/heads/' + BRANCH_NAME_REAL]
    ],
    userRemoteConfigs: [[credentialsId: 'Github_User_And_Token', url: 'https://github.com/LORD-MicroStrain/mip_sdk.git']],
    extensions: [
    ]
  ])
}

pipeline {
  agent none
  options {
    // Set a timeout for the whole pipeline. The timer starts when the project is queued
    timeout(time: 1, unit: 'HOURS')
    // Only keep this number of builds for the job
    buildDiscarder(logRotator(numToKeepStr: "10"))
  }
  stages {
    stage('Build') {
      // Run all the builds in parallel
      parallel {
        stage('Documentation') {
          agent { label 'linux-amd64' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64 --docs"
              archiveArtifacts artifacts: 'build_docs/mipsdk_*'
            }
          }
        }
        stage('Windows x86') {
          agent { label 'windows10' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              powershell """
                mkdir build_Win32
                cd build_Win32
                cmake .. -A "Win32" -DMICROSTRAIN_BUILD_EXAMPLES=ON -DMICROSTRAIN_BUILD_PACKAGE=ON
                cmake --build . --config Release
                cmake --build . --config Release --target package
              """
              archiveArtifacts artifacts: 'build_Win32/mipsdk_*'
            }
          }
        }
        stage('Windows x64') {
          agent { label 'windows10' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              powershell """
                mkdir build_x64
                cd build_x64
                cmake .. -DMICROSTRAIN_BUILD_EXAMPLES=ON -DMICROSTRAIN_BUILD_PACKAGE=ON
                cmake --build . --config Release
                cmake --build . --config Release --target package
              """
              archiveArtifacts artifacts: 'build_x64/mipsdk_*'
            }
          }
        }
        stage('Ubuntu amd64') {
          agent { label 'linux-amd64' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64"
              archiveArtifacts artifacts: 'build_ubuntu_amd64/mipsdk_*'
            }
          }
        }
        stage('Ubuntu arm64') {
          agent { label 'linux-arm64' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh "./.devcontainer/docker_build.sh --os ubuntu --arch arm64v8"
              archiveArtifacts artifacts: 'build_ubuntu_arm64v8/mipsdk_*'
            }
          }
        }
        stage('Ubuntu arm32') {
          agent { label 'linux-arm64' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh "./.devcontainer/docker_build.sh --os ubuntu --arch arm32v7"
              archiveArtifacts artifacts: 'build_ubuntu_arm32v7/mipsdk_*'
            }
          }
        }
        stage("Mac M2") {
          agent { label 'mac-m2' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh '''
                mkdir build_mac_arm64
                cd build_mac_arm64
                cmake .. -DMICROSTRAIN_BUILD_EXAMPLES=ON -DMICROSTRAIN_BUILD_PACKAGE=ON -DCMAKE_BUILD_TYPE=RELEASE
                cmake --build . -j $(sysctl -n hw.ncpu)
                cmake --build . --target package
              '''
              archiveArtifacts artifacts: 'build_mac_arm64/mipsdk_*'
            }
          }
        }
        stage("Mac Intel") {
          agent { label 'mac-intel' }
          options {
            skipDefaultCheckout()
            timeout(time: 5, activity: true, unit: 'MINUTES')
          }
          steps {
            script {
              checkoutRepo()
              env.setProperty('BRANCH_NAME', branchName())
              sh '''
                mkdir build_mac_intel
                cd build_mac_intel
                cmake .. -DMICROSTRAIN_BUILD_EXAMPLES=ON -DMICROSTRAIN_BUILD_PACKAGE=ON -DCMAKE_BUILD_TYPE=RELEASE
                cmake --build . -j $(sysctl -n hw.ncpu)
                cmake --build . --target package
              '''
              archiveArtifacts artifacts: 'build_mac_intel/mipsdk_*'
            }
          }
        }
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
