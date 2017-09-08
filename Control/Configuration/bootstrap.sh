printf "\nFixing vagrant authentication issue.\n\n"
wget -q https://raw.githubusercontent.com/mitchellh/vagrant/master/keys/vagrant.pub -O ~/.ssh/authorized_keys
chmod 700 ~/.ssh/
chmod 600 ~/.ssh/authorized_keys
chown -R vagrant:vagrant ~/.ssh/

printf "\nDownloading apt-wait script...\n\n"
sudo wget -q https://raw.githubusercontent.com/MST-MRR/IARC-2018/master/Control/Configuration/apt-get-wait.sh -O /usr/local/sbin/apt-get
sudo chmod +x /usr/local/sbin/apt-get

printf "\nAdding MRR user.\n\n"
sleep 5s
sudo adduser mrr
sudo adduser mrr sudo
sudo adduser mrr dialout

printf "\nRemove unnecessary packages...\n\n"
sleep 5s
sudo apt-get install nano curl cmake automake autoconf
sudo apt-get remove -y --purge libreoffice*
sudo apt-get clean
sudo apt-get autoremove -y

printf "\nRemoving vagrant user from login...\n\n"
sleep 1s
sudo sed -i 's/SystemAccount=false/SystemAccount=true/g' /var/lib/AccountsService/users/vagrant

printf "\nDownloading provisioning script...\n\n"
sudo wget -q https://raw.githubusercontent.com/MST-MRR/IARC-2018/master/Control/Configuration/setup_vm.sh -O /home/mrr/setup_vm.sh
sudo chmod +x /home/mrr/setup_vm.sh
sudo chown mrr:mrr /home/mrr/setup_vm.sh

printf "\nDone.\n\n"
