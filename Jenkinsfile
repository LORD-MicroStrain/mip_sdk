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
            cleanWs()
            checkout scm
            powershell """
              mkdir build_Win32
              cd build_Win32
              cmake .. `
                -A "Win32" `
                -DBUILD_DOCUMENTATION=ON `
                -DBUILD_PACKAGE=ON
              cmake --build . -j
              cmake --build . --target package
              cmake --build . --target package_docs
            """
            archiveArtifacts artifacts: 'build_Win32/mipsdk_*'
          }
        }
        stage('Windows x64') {
          agent { label 'windows10' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            powershell """
              mkdir build_x64
              cd build_x64
              cmake .. `
                -A "x64" `
                -DBUILD_PACKAGE=ON
              cmake --build . -j
              cmake --build . --target package
            """
            archiveArtifacts artifacts: 'build_x64/mipsdk_*'
          }
        }
        stage('Ubuntu amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            sh "cp /etc/pki/ca-trust/source/anchors/ZScaler.crt ./.devcontainer/extra_cas/"
            sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64"
            archiveArtifacts artifacts: 'build_ubuntu_amd64/mipsdk_*'
          }
        }
        stage('Centos amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            cleanWs()
            checkout scm
            sh "cp /etc/pki/ca-trust/source/anchors/ZScaler.crt ./.devcontainer/extra_cas/"
            sh "./.devcontainer/docker_build.sh --os centos --arch amd64"
            archiveArtifacts artifacts: 'build_centos_amd64/mipsdk_*'
          }
        }
      }
    }
  }
  post {
    success {
      script {
        def repo = "LORD-MicroStrain/libmip"
        if (BRANCH_NAME && BRANCH_NAME == 'develop') {
          node("linux-amd64") {
            withCredentials([string(credentialsId: 'MICROSTRAIN_BUILD_GH_TOKEN', variable: 'GH_TOKEN')]) {
              sh '''
              release_name="latest"
              artifacts=$(find "${WORKSPACE}/../builds/${BUILD_NUMBER}/archive/" -type f)
              gh release delete \
                -y \
                -R "${repo}" \
                "${release_name}"
              gh release create \
                -R "${repo}" \
                --title "${release_name}" \
                --target "${BRANCH_NAME}" \
                --generate-notes \
                "${release_name}" ${artifacts}
              '''
            }
          }
        } else if (BRANCH_NAME && BRANCH_NAME == 'master_test') {
          node("linux-amd64") {
            withCredentials([string(credentialsId: 'MICROSTRAIN_BUILD_GH_TOKEN', variable: 'GH_TOKEN')]) {
              sh '''
              # Release to the latest version if the master commit matches up with the commit of that version
              latest_version=$(gh release list --exclude-drafts -R LORD-MicroStrain/libmip | tr '\t' ' ' | tr -s ' ' | cut -d' ' -f1 | grep -v "latest" | sort -V -r | head -1)
              current_commit=$(git rev-list -n 1 ${BRANCH_NAME})
              latest_version_commit=$(git rev-list -n 1 "${latest_version}")
              artifacts=$(find "${WORKSPACE}/../builds/${BUILD_NUMBER}/archive/" -type f)
              if [[ "${current_commit}" == "${latest_version_commit}" ]]; then
                gh release delete \
                  -y \
                  -R "${repo}" \
                  "${latest_version}"
                gh release create \
                  -R "${repo}" \
                  --title "${latest_version}" \
                  --target "${latest_version}" \
                  --notes "" \
                  "${latest_version}" ${artifacts}
              else
                echo "Not releasing from ${BRANCH_NAME} since the current commit does not match the latest released version commit"
                echo "${BRANCH_NAME} commit: ${current_commit}"
                echo "${latest_version} commit: ${latest_version_commit}"
              fi
              '''
            }
          }
        }
      }
    }
  }
}