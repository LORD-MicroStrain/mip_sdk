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
                mkdir build
                cd build
                cmake .. `
                    -A "Win32" `
                    -DBUILD_DOCUMENTATION=ON `
                    -DBUILD_PACKAGE=ON
                cmake --build . -j
                #cmake --build . --target package
                #cmake --build . --target package_docs
            """
            archiveArtifacts artifacts: 'build/mipsdk_*'
          }
        }
      }
    }
  }
}