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
    userRemoteConfigs: [[credentialsId: 'ffc94480-3383-4390-82e6-af2fb5e6c76d', url: 'https://github.com/LORD-MicroStrain/libmip.git']],
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
            checkoutRepo()
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
            checkoutRepo()
            sh "cp /etc/pki/ca-trust/source/anchors/ZScaler.crt ./.devcontainer/extra_cas/"
            sh "./.devcontainer/docker_build.sh --os ubuntu --arch amd64"
            archiveArtifacts artifacts: 'build_ubuntu_amd64/mipsdk_*'
          }
        }
        stage('Centos amd64') {
          agent { label 'linux-amd64' }
          options { skipDefaultCheckout() }
          steps {
            checkoutRepo()
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
        if (BRANCH_NAME && BRANCH_NAME == 'develop_test') {
          node("linux-amd64") {
            withCredentials([string(credentialsId: 'MICROSTRAIN_BUILD_GH_TOKEN', variable: 'GH_TOKEN')]) {
              sh '''
              release_name="develop"
              repo="LORD-MicroStrain/libmip"
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

              # Commit the documentation to the github pages branch
              export GIT_ASKPASS="${HOME}/.git-askpass"
              docs_zip=$(find "${archive_dir}" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)
              docs_dir="${WORKSPACE}/mip_sdk_documentation/${release_name}"
              git clone -b "main" "https://github.com/LORD-MicroStrain/mip_sdk_documentation.git" mip_sdk_documentation
              rm -rf "${docs_dir}"
              mkdir -p "${docs_dir}"
              pushd "${docs_dir}"
              unzip "${docs_zip}" -d "${docs_dir}"
              git add --all
              git commit -m "Adds documentation for ${release_name}"
              git push origin main
              popd
              '''
            }
          }
        } else if (BRANCH_NAME && BRANCH_NAME == 'master') {
          node("linux-amd64") {
            withCredentials([string(credentialsId: 'MICROSTRAIN_BUILD_GH_TOKEN', variable: 'GH_TOKEN')]) {
              sh '''
              # Release to the latest version if the master commit matches up with the commit of that version
              repo="LORD-MicroStrain/libmip"
              archive_dir="${WORKSPACE}/../builds/${BUILD_NUMBER}/archive/"
              artifacts=$(find "${archive_dir}" -type f)
              if git describe --exact-match --tags HEAD &> /dev/null; then
                # Deploy the artifacts to Github
                tag=$(git describe --exact-match --tags HEAD)
                gh release delete \
                  -y \
                  -R "${repo}" \
                  "${tag}" || echo "No existing release named ${tag}"
                gh release create \
                  -R "${repo}" \
                  --title "${tag}" \
                  --target "${BRANCH_NAME}" \
                  --notes "" \
                  "${tag}" ${artifacts}
                
                # Commit the documentation to the github pages branch
                export GIT_ASKPASS="${HOME}/.git-askpass"
                docs_zip=$(find "${archive_dir}" -type f -name "mipsdk_*_Documentation.zip" | sort | uniq)
                docs_dir="${WORKSPACE}/mip_sdk_documentation/${tag}"
                git clone -b "main" "https://github.com/LORD-MicroStrain/mip_sdk_documentation.git" mip_sdk_documentation
                rm -rf "${docs_dir}"
                mkdir -p "${docs_dir}"
                pushd "${docs_dir}"
                unzip "${docs_zip}" -d "${docs_dir}"
                git add --all
                git commit -m "Adds documentation for ${tag}"
                git push origin main
                popd
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